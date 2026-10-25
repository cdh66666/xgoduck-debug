"""Split the existing trunk assembly mesh; never substitute guessed part positions.

Geometry and total/battery/extension mass are known. UNO Q net mass and printing
mass distribution remain assumptions, explicitly recorded in the generated JSON.
This changes inertials only; the original model and all contact geometry remain.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import os
from pathlib import Path
import struct
import xml.etree.ElementTree as ET
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def triangles(path):
    if path.suffix == '.obj':
        vertices, faces = [], []
        for line in path.read_text().splitlines():
            row = line.split()
            if not row: continue
            if row[0] == 'v': vertices.append([float(x) for x in row[1:4]])
            if row[0] == 'f':
                indices = [int(x.split('/')[0])-1 for x in row[1:]]
                faces.extend((indices[0],indices[i],indices[i+1]) for i in range(1,len(indices)-1))
        return np.asarray(vertices)[np.asarray(faces)]
    data = path.read_bytes()
    count = struct.unpack_from('<I',data,80)[0]
    if len(data) != 84+50*count: raise ValueError('Expected binary STL: '+str(path))
    records = np.frombuffer(data,dtype=np.uint8,offset=84).reshape(count,50)
    return records[:,12:48].copy().view('<f4').astype(float).reshape(count,3,3)


def components(tris):
    vertices, indices = np.unique(np.round(tris.reshape(-1,3),7),axis=0,return_inverse=True)
    faces = indices.reshape(-1,3); parent = np.arange(len(vertices))
    def find(a):
        while parent[a] != a: parent[a]=parent[parent[a]]; a=parent[a]
        return int(a)
    for a,b,c in faces:
        root=find(a); parent[find(b)]=root; parent[find(c)]=root
    labels=np.asarray([find(a) for a in faces[:,0]])
    pieces=[tris[labels==i] for i in np.unique(labels)]
    return sorted(pieces,key=lambda t:properties(t)[0],reverse=True)


def properties(tris):
    """Signed tetrahedron integration, returning volume/COM/inertia per kg."""
    a,b,c=tris[:,0],tris[:,1],tris[:,2]
    volumes=np.einsum('ij,ij->i',a,np.cross(b,c))/6
    if volumes.sum()<0: volumes=-volumes
    volume=float(volumes.sum())
    if volume <= 1e-12: raise ValueError('Zero-volume component')
    sums=a+b+c; center=(volumes[:,None]*sums).sum(0)/(4*volume)
    second=sum(np.einsum('ni,nj->nij',x,x) for x in (sums,a,b,c))
    second=(volumes[:,None,None]*second).sum(0)/20
    inertia=np.trace(second)*np.eye(3)-second
    inertia-=volume*(np.dot(center,center)*np.eye(3)-np.outer(center,center))
    inertia/=volume
    if np.linalg.eigvalsh(inertia).min() <= 0: raise ValueError('Invalid component inertia')
    return volume,center,inertia


def combine(parts):
    mass=sum(p['mass_kg'] for p in parts)
    center=sum(p['mass_kg']*p['center'] for p in parts)/mass
    inertia=np.zeros((3,3))
    for p in parts:
        delta=p['center']-center
        inertia+=p['mass_kg']*(p['inertia_per_kg']+np.dot(delta,delta)*np.eye(3)-np.outer(delta,delta))
    return mass,center,inertia


def inertial(parent,mass,center,inertia):
    if np.linalg.eigvalsh(inertia).min() <= 0: raise ValueError('Invalid assembled inertia')
    full=[inertia[0,0],inertia[1,1],inertia[2,2],inertia[0,1],inertia[0,2],inertia[1,2]]
    return ET.SubElement(parent,'inertial',pos=' '.join(f'{v:.12g}' for v in center),
        mass=f'{mass:.12g}',fullinertia=' '.join(f'{v:.12g}' for v in full))


def build(output,uno_g=30.,servo_g=21.):
    source=ROOT/'model/robot.xml'; assets=source.parent/'assets'
    tree=ET.parse(source); trunk=tree.find("./worldbody/body[@name='trunk_base']")
    old=trunk.find('inertial'); original_mass=float(old.get('mass'))
    old_center=[float(x) for x in old.get('pos').split()]
    original_total=sum(float(x.get('mass')) for x in tree.findall('.//inertial'))
    trunk_budget=.826-(original_total-original_mass)
    pieces=components(triangles(assets/'trunk_base.stl'))
    if len(pieces)!=6: raise ValueError('Assembly changed; inspect components before assigning masses')
    # Reviewed component: the 18 x 36.8 x 66 mm two-cell pack already in the STL.
    battery=pieces[1]
    size=battery.max(axis=(0,1))-battery.min(axis=(0,1))
    if not np.allclose(size,[.018,.036817,.066],atol=.0005): raise ValueError('Battery geometry changed')
    parts=[]
    def add(name,tris,mass,source_note):
        volume,center,tensor=properties(tris)
        parts.append(dict(name=name,mass_kg=mass,center=center,inertia_per_kg=tensor,
                          volume_m3=volume,source=source_note))
    add('battery',battery,.108,'User: battery about 108 g; existing two-cell assembly geometry; uniform component density')
    add('trunk_servos',pieces[0],3*servo_g/1000,'Three parent-mounted servos; HD-1910 nominal 21 +/-2 g each; uniform assembly density approximation')
    add('uno_q',triangles(assets/'uno-q.stl'),uno_g/1000,'Unverified board-only mass prior, default 30 g; test 20..40 g; geometry from existing UNO Q mesh')
    add('extension',triangles(assets/'extension.obj'),.020,'User: extension board about 20 g; uniform existing mesh density')
    bearings=np.concatenate(pieces[4:6]);bearing_volume=properties(bearings)[0]
    add('bearings',bearings,bearing_volume*7850,'Existing ring geometry; homogeneous steel density approximation 7850 kg/m3')
    printed=np.concatenate([pieces[2],pieces[3],triangles(assets/'cover.stl')])
    printed_mass=trunk_budget-sum(p['mass_kg'] for p in parts)
    if printed_mass <= 0: raise ValueError('Component masses exceed measured total mass budget')
    add('printed_trunk',printed,printed_mass,'Residual trunk mass budget; uniform effective density across actual printed mesh; PETG 15% infill is not a measured component mass')
    mass,center,tensor=combine(parts)
    trunk.remove(old)
    # The root owns printed material, the fixed children own the hardware.
    p=parts[-1];inertial(trunk,p['mass_kg'],p['center'],p['mass_kg']*p['inertia_per_kg'])
    for p in parts[:-1]:
        body=ET.SubElement(trunk,'body',name='mass_'+p['name'])
        inertial(body,p['mass_kg'],p['center'],p['mass_kg']*p['inertia_per_kg'])
    tree.getroot().set('model','xgoduck_inertial_candidate_20261025')
    # Permit output under outputs/ without copying the actual source meshes.
    tree.find('compiler').set('meshdir',os.path.relpath(assets.resolve(),output.resolve().parent).replace('\\','/'))
    ET.indent(tree,space='  ');output.parent.mkdir(parents=True,exist_ok=True)
    if output.exists():raise FileExistsError(output)
    tree.write(output,encoding='utf-8',xml_declaration=True)
    report=dict(status='EXPERIMENTAL_INERTIAL_CANDIDATE_NOT_HARDWARE_VALIDATED',
        source_xml_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        xml=str(output.resolve()),xml_sha256=hashlib.sha256(output.read_bytes()).hexdigest(),
        total_mass_kg=.826,source_total_mass_kg=original_total,
        original_trunk_mass_kg=original_mass,original_trunk_center_m=old_center,
        new_trunk_mass_kg=mass,new_trunk_center_m=center.tolist(),new_trunk_inertia_kg_m2=tensor.tolist(),
        parameters=dict(uno_q_mass_g=uno_g,servo_mass_g=servo_g),
        parts=[{k:(v.tolist() if isinstance(v,np.ndarray) else v) for k,v in p.items()} for p in parts],
        unchanged='All contact geometry, other articulated-body inertials, policy HOME and actuator parameters',
        limitations=['Other body masses retain prior CAD lumped inertials, not newly weighed PETG parts',
            'Board-only UNO Q mass and printed density require sensitivity testing',
            'Uniform-density mesh approximation does not identify individual motor rotor COM',
            'This is a dynamics candidate, not a new mechanical design or a structure-strength validation'],
        sources={'uno_q_geometry':'https://docs.arduino.cc/resources/datasheets/ABX00162-datasheet.pdf',
                 'servo_nominal':'https://ftservo.github.io/ftservo-wiki/products/models/hd-1910-c001/main/'})
    import mujoco
    model=mujoco.MjModel.from_xml_path(str(output.resolve()))
    report['compiled_total_mass_kg']=float(model.body_mass.sum())
    if abs(report['compiled_total_mass_kg']-.826)>1e-8:raise ValueError('Compiled mass mismatch')
    output.with_suffix('.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--uno-mass-g',type=float,default=30.)
    parser.add_argument('--servo-mass-g',type=float,default=21.)
    args=parser.parse_args();report=build(args.output,args.uno_mass_g,args.servo_mass_g)
    print(json.dumps({k:report[k] for k in ('status','xml','xml_sha256','compiled_total_mass_kg','original_trunk_center_m','new_trunk_center_m')},indent=2))
