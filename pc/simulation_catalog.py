"""Read-only bindings for the local physics sandbox; no robot imports."""
from pathlib import Path
import hashlib
import xml.etree.ElementTree as ET

RUN = 'r5-noeva-velocity-20261020'
FILES = {'walk':'xgoduck_walk.onnx', 'getup':'xgoduck_getup.onnx',
         'pick':'xgoduck_pick.onnx', 'roulade':'xgoduck_roulade.onnx'}


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def resolve_model(root, model_id):
    from active_runtime import runtime_manifest, runtime_source
    root = Path(root).resolve()
    if model_id not in {'official-'+name for name in FILES}:
        raise ValueError('请选择当前官方模型库中的模型')
    name = model_id.removeprefix('official-')
    manifest = runtime_manifest(root)
    models = {}
    for key, filename in FILES.items():
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
