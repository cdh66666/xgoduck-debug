"""Read-only bindings for the local physics sandbox; no robot imports."""
from pathlib import Path
import hashlib
import json
import xml.etree.ElementTree as ET

RUN = 'r5-noeva-velocity-20261020'
FILES = {'walk':'xgoduck_walk.onnx', 'getup':'xgoduck_getup.onnx',
         'pick':'xgoduck_pick.onnx', 'roulade':'xgoduck_roulade.onnx',
         'fast':'xgoduck_fast_forward.onnx', 'navigation':'xgoduck_walk.onnx'}

NAVIGATION_POINTS = [[-.3,0,0],[0,0,1],[0,0,-1]]


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def retained_navigation(manifest, walk):
    """Only the deployed, hash-bound admission may expose retained walk as nav."""
    extension=manifest.get('keyboard_extension')
    admission=manifest.get('keyboard_admission')
    if not isinstance(extension,dict) or not isinstance(admission,dict) or not walk:
        return None
    scope=admission.get('navigation_scope',{})
    if not isinstance(scope,dict):
        return None
    if (extension.get('schema')!='xgoduck-keyboard-extension-active-v1' or
            admission.get('schema')!='xgoduck-keyboard-navigation-admission-v1' or
            admission.get('result')!='PASS_LIMITED_KEYBOARD_SCOPE' or
            admission.get('limited_forward_deployable_offline') is not True or
            admission.get('manual_pick_deployable_offline') is not True or
            admission.get('navigation_deployable_offline') is not True or
            admission.get('all_core_deployable_offline') is not False or
            admission.get('original_walk_sha256')!=walk['sha256'] or
            scope.get('mode')!='retained_official' or
            scope.get('artifact_sha256')!=walk['sha256'] or
            scope.get('command_points')!=NAVIGATION_POINTS):
        return None
    # runtime_manifest verifies this chain before publishing keyboard_admission;
    # keep the selected sandbox artifact bound to the same exact receipt bytes.
    try:
        path=Path(extension['admission_path'])
        receipt=Path(extension['receipt_path'])
        digest=extension['admission_sha256']
        if (not path.is_absolute() or not receipt.is_absolute() or sha256(path)!=digest or
                sha256(receipt)!=extension['receipt_sha256']):
            raise ValueError('导航准入文件与部署绑定不符')
        deployed=json.loads(receipt.read_text('utf-8'))
        if (deployed.get('admission_sha256')!=digest or
                deployed.get('ok') is not True or deployed.get('phase')!='activated' or
                json.loads(path.read_text('utf-8'))!=admission):
            raise ValueError('导航准入未绑定当前激活回执')
    except (KeyError,OSError,TypeError) as error:
        raise ValueError('导航准入绑定不可用') from error
    return {**walk,'label':'原版后退 / 转向','source_model':'walk',
            'default_command':[-.3,0,0],'command_points':NAVIGATION_POINTS,
            'physical_acceptance':scope.get('physical_acceptance','pending'),
            'all_core_deployable_offline':admission['all_core_deployable_offline'],
            'navigation_scope':dict(scope),
            'note':'S 指令 -0.3；离线后退速度约 0.18-0.22 m/s，完整速度跟踪未通过，实物待验。'}


def resolve_model(root, model_id):
    from active_runtime import runtime_manifest, runtime_source
    root = Path(root).resolve()
    if model_id not in {'official-'+name for name in FILES}:
        raise ValueError('请选择当前官方模型库中的模型')
    name = model_id.removeprefix('official-')
    manifest = runtime_manifest(root)
    models = {}
    for key, filename in FILES.items():
        if key=='navigation':
            continue
        entry = 'python/'+filename
        if entry not in manifest['new_source_manifest']:
            continue
        path = runtime_source(root, entry, manifest).resolve()
        if not path.is_file():
            raise ValueError('本地仿真权重未下载；先运行 python pc/tools/fetch_robot_models.py --fetch（只读取鸭子，不改变设备）')
        digest = manifest['new_source_manifest'][entry]
        if sha256(path) != digest:
            raise ValueError('本地模型 SHA 与当前运行清单不符：'+filename)
        models[key] = {'path': str(path), 'sha256': digest}
    navigation=retained_navigation(manifest,models.get('walk'))
    if navigation:
        models['navigation']=navigation
    if name not in models:
        raise ValueError('此模型尚未部署到当前固定运行链')
    sources = {}
    for key in ('policy.py', 'rl_core.py'):
        entry='python/'+key
        path=runtime_source(root, entry, manifest).resolve(strict=True)
        if sha256(path) != manifest['new_source_manifest'][entry]:
            raise ValueError('官方推理源 SHA 不匹配：'+key)
        sources[key]={'path':str(path),'sha256':sha256(path)}
    xml = root.parent/'simulation-model/robot.xml'
    recovery = runtime_source(root,'python/recovery.py',manifest)
    bam = root.parent/'simulation-model/1910_m6.json'
    return {'model_id':model_id, 'selected':name, 'path':models[name]['path'],
            'sha256':models[name]['sha256'], 'models':models, 'sources':sources,
            'xml':str(xml.resolve(strict=True)), 'xml_sha256':sha256(xml),
            'recovery':str(recovery.resolve(strict=True)), 'recovery_sha256':sha256(recovery),
            'bam':str(bam.resolve(strict=True)), 'bam_sha256':sha256(bam)}


def geometry(root, model_id):
    bound = resolve_model(root, model_id)
    xml = Path(bound['xml'])
    tree = ET.parse(xml).getroot()
    parents = {child:parent for parent in tree.iter() for child in parent}
    meshes = {m.get('name',Path(m.get('file')).stem):m for m in tree.findall('./asset/mesh')}
    records=[]
    for geom in tree.findall('.//geom'):
        if geom.get('class') != 'visual' or not geom.get('mesh'):
            continue
        mesh=meshes[geom.get('mesh')]
        filename=mesh.get('file')
        if Path(filename).name != filename:
            raise ValueError('模型网格路径不符合当前资源目录')
        records.append({'id':geom.get('mesh'),'body':parents[geom].get('name'),
                        'file':'/model-replay-assets/assets/'+filename,
                        'mesh_scale':[float(v) for v in mesh.get('scale','1 1 1').split()],
                        'geom_position_m':[float(v) for v in geom.get('pos','0 0 0').split()],
                        'geom_quaternion_wxyz':[float(v) for v in geom.get('quat','1 0 0 0').split()],
                        'color':'#2f9574' if geom.get('mesh')=='uno-q' else '#98a6ab'})
    return {'version':RUN, 'records':records,'units':'m', 'modelSha':bound['sha256'],
            'xmlSha':bound['xml_sha256'], 'evidence_kind':'interactive_physics'}
