"""Decrypt the repository's connection bundle locally; no device/network IO."""
from __future__ import annotations
import argparse,csv,getpass,hashlib,io,json,os,secrets,stat,subprocess,sys,tempfile,zipfile
from pathlib import Path

MAGIC=b'XGODKEY1'
ITERATIONS=600_000
MAX_BYTES=256*1024
FILES=('device.local.json','tools/.private/ssh-identity.json',
       'tools/.private/id_ed25519','tools/.private/known_hosts',
       'tools/.private/wireless-calibration-auth.json','qa/wireless-connection.json')

def _key(password,salt):
    from cryptography.hazmat.primitives import hashes
    from cryptography.hazmat.primitives.kdf.pbkdf2 import PBKDF2HMAC
    return PBKDF2HMAC(algorithm=hashes.SHA256(),length=32,salt=salt,iterations=ITERATIONS).derive(password.encode('utf-8'))

def encrypt_bundle(raw,password):
    from cryptography.hazmat.primitives.ciphers.aead import AESGCM
    if not 12<=len(password)<=1024:raise ValueError('连接包口令长度不符合要求')
    salt=secrets.token_bytes(16);nonce=secrets.token_bytes(12);header=MAGIC+salt+nonce
    return header+AESGCM(_key(password,salt)).encrypt(nonce,raw,header)

def decrypt_bundle(blob,password):
    from cryptography.exceptions import InvalidTag
    from cryptography.hazmat.primitives.ciphers.aead import AESGCM
    if not 52<=len(blob)<=MAX_BYTES or blob[:8]!=MAGIC:raise ValueError('连接包格式不正确')
    try:return AESGCM(_key(password,blob[8:24])).decrypt(blob[24:36],blob[36:],blob[:36])
    except InvalidTag:raise ValueError('口令不正确或连接包已损坏；未导入任何配置') from None

def entries_from_zip(raw):
    if len(raw)>MAX_BYTES:raise ValueError('解密后的连接包过大')
    with zipfile.ZipFile(io.BytesIO(raw)) as archive:
        infos=archive.infolist()
        if len(infos)!=len(FILES) or {i.filename for i in infos}!=set(FILES):raise ValueError('连接包文件范围不正确')
        if any(i.is_dir() or stat.S_ISLNK(i.external_attr>>16) or i.file_size>65536 for i in infos):raise ValueError('连接包文件类型或大小不正确')
        entries={name:archive.read(name) for name in FILES}
    device=json.loads(entries['device.local.json']);identity=json.loads(entries['tools/.private/ssh-identity.json'])
    auth=json.loads(entries['tools/.private/wireless-calibration-auth.json'])
    if not device.get('serial') or device['serial']!=identity.get('serial'):raise ValueError('连接包设备身份不一致')
    if identity.get('key_path')!='id_ed25519' or identity.get('known_hosts_path')!='known_hosts':raise ValueError('连接包SSH文件路径不正确')
    if not isinstance(auth.get('key'),str) or not auth['key']:raise ValueError('连接包缺少板端认证')
    return entries

def _restrict_private_files(paths):
    if os.name=='nt':
        row=next(csv.reader(subprocess.check_output(['whoami','/user','/fo','csv','/nh']).decode('utf-8',errors='replace').strip().splitlines()))
        sid=row[-1].strip()
        if not sid.startswith('S-1-'):raise RuntimeError('无法读取当前Windows用户SID')
        for path in paths:
            subprocess.run(['icacls',str(path),'/inheritance:r','/grant:r','*'+sid+':F'],check=True,stdout=subprocess.DEVNULL,stderr=subprocess.PIPE)
    else:
        for path in paths:path.chmod(0o600)

def install_bundle(blob,password,target,*,replace=False,permissions=True):
    entries=entries_from_zip(decrypt_bundle(blob,password))
    target=Path(target).resolve();paths={name:target/name for name in FILES}
    for path in paths.values():
        if not path.resolve().is_relative_to(target) or path.is_symlink():raise ValueError('配置目标超出调试台目录')
    conflicts=[name for name,path in paths.items() if path.exists() and path.read_bytes()!=entries[name]]
    if conflicts and not replace:raise ValueError('已有不同连接配置，未覆盖；备份后可使用 --replace')
    target.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.connection-import-',dir=target) as temporary:
        for index,(name,raw) in enumerate(entries.items()):
            source=Path(temporary)/str(index);source.write_bytes(raw)
            path=paths[name];path.parent.mkdir(parents=True,exist_ok=True)
            if not path.exists() or path.read_bytes()!=raw:os.replace(source,path)
    if permissions:_restrict_private_files([path for name,path in paths.items() if '/.private/' in name])
    proof={'schema':'xgoduck-imported-connection-v1','bundle_sha256':hashlib.sha256(blob).hexdigest(),'files':len(entries),'network_requests':0,'robot_actions':0}
    (target/'tools/.private/connection-import.json').write_text(json.dumps(proof,indent=2)+'\n',encoding='utf-8')
    return proof

def main():
    parser=argparse.ArgumentParser(description='导入仓库内的加密连接配置，不连接或操作机器人。')
    parser.add_argument('--bundle',type=Path,default=Path(__file__).resolve().parents[2]/'connection/private-connection.enc')
    parser.add_argument('--replace',action='store_true',help='明确覆盖已有的不同连接配置')
    args=parser.parse_args()
    if args.bundle.stat().st_size>MAX_BYTES:raise ValueError('连接包过大')
    password=os.environ.pop('XGODUCK_CONNECTION_PASSWORD',None) or getpass.getpass('连接配置解密口令：')
    proof=install_bundle(args.bundle.read_bytes(),password,Path(__file__).resolve().parents[1],replace=args.replace)
    print('连接配置已导入（%s个文件）；未连接或操作机器人。'%proof['files'])

if __name__=='__main__':
    try:main()
    except (ValueError,OSError,zipfile.BadZipFile,RuntimeError,subprocess.CalledProcessError) as error:
        print('导入失败：'+str(error),file=sys.stderr);raise SystemExit(1)
    except ImportError:
        print('请先安装依赖：python -m pip install -r pc/requirements.txt',file=sys.stderr);raise SystemExit(1)
