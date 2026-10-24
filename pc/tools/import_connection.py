"""Install the public device-instance connection profile; no network/device IO."""
from __future__ import annotations
import argparse,csv,hashlib,json,os,subprocess,sys,tempfile
from pathlib import Path
MAPPING={'device.json':'device.local.json',
 'ssh-identity.json':'tools/.private/ssh-identity.json',
 'id_ed25519':'tools/.private/id_ed25519',
 'known_hosts':'tools/.private/known_hosts',
 'wireless-auth.json':'tools/.private/wireless-calibration-auth.json',
 'network.json':'qa/wireless-connection.json'}
def read_profile(folder):
    folder=Path(folder);entries={}
    for source,target in MAPPING.items():
        path=folder/source
        if not path.is_file() or not 0<path.stat().st_size<=65536:raise ValueError('连接配置缺失或大小不正确：'+source)
        entries[target]=path.read_bytes()
    device=json.loads(entries['device.local.json']);identity=json.loads(entries['tools/.private/ssh-identity.json'])
    auth=json.loads(entries['tools/.private/wireless-calibration-auth.json'])
    if not device.get('serial') or device['serial']!=identity.get('serial'):raise ValueError('连接配置设备身份不一致')
    if identity.get('key_path')!='id_ed25519' or identity.get('known_hosts_path')!='known_hosts':raise ValueError('SSH配置应使用profile内的固定文件名')
    if not isinstance(auth.get('key'),str) or not auth['key']:raise ValueError('缺少板端认证')
    return entries

def _restrict_private_files(paths):
    if os.name=='nt':
        row=next(csv.reader(subprocess.check_output(['whoami','/user','/fo','csv','/nh']).decode('utf-8',errors='replace').strip().splitlines()))
        sid=row[-1].strip()
        if not sid.startswith('S-1-'):raise RuntimeError('无法读取当前Windows用户SID')
        for path in paths:
            # OpenSSH rejects an explicit OWNER RIGHTS ACE even for this owner.
            subprocess.run(['icacls',str(path),'/inheritance:r','/grant:r','*'+sid+':F',
                            '/remove:g','*S-1-3-4'],check=True,stdout=subprocess.DEVNULL,stderr=subprocess.PIPE)
    else:
        for path in paths:path.chmod(0o600)


def install_profile(profile,target,*,replace=False,permissions=True):
    entries=read_profile(profile);target=Path(target).resolve();paths={name:target/name for name in entries}
    for path in paths.values():
        if not path.resolve().is_relative_to(target) or path.is_symlink():raise ValueError('配置目标超出调试台目录')
    conflicts=[name for name,path in paths.items() if path.exists() and path.read_bytes()!=entries[name]]
    if conflicts and not replace:raise ValueError('已有不同连接配置，未覆盖；备份后可使用 --replace')
    target.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.connection-import-',dir=target) as temporary:
        for index,(name,raw) in enumerate(entries.items()):
            source=Path(temporary)/str(index);source.write_bytes(raw);path=paths[name];path.parent.mkdir(parents=True,exist_ok=True)
            if not path.exists() or path.read_bytes()!=raw:os.replace(source,path)
    if permissions:_restrict_private_files([p for n,p in paths.items() if '/.private/' in n])
    proof={'schema':'xgoduck-public-profile-import-v1','files':len(entries),'profile_mode':'plain_device_instance','network_requests':0,'robot_actions':0}
    (target/'tools/.private/connection-import.json').write_text(json.dumps(proof,indent=2),encoding='utf-8')
    return proof

def main():
    parser=argparse.ArgumentParser(description='直接导入仓库内设备实例配置，不需要口令，不连接或操作机器人。')
    parser.add_argument('--profile',type=Path,default=Path(__file__).resolve().parents[2]/'connection/profile')
    parser.add_argument('--replace',action='store_true',help='明确覆盖已有的不同连接配置')
    args=parser.parse_args();proof=install_profile(args.profile,Path(__file__).resolve().parents[1],replace=args.replace)
    print('连接配置已直接导入（%s个文件）；无需解密，未连接或操作机器人。'%proof['files'])
if __name__=='__main__':
    try:main()
    except (ValueError,OSError,RuntimeError,subprocess.CalledProcessError) as error:
        print('导入失败：'+str(error),file=sys.stderr);raise SystemExit(1)
