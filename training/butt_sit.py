"""Butt-supported sit research. Native 50 Hz policy / 200 Hz BAM; no hardware IO."""
import json,math
from pathlib import Path
import numpy as np
from sit_training import Plant,HOME,INDEX,smooth

ROOT=Path(__file__).resolve().parents[1]
XML=ROOT/'training/model/robot_inertial_candidate_20261025.xml'
SEED_RECORD=ROOT/'training/model/standing_seed_20261025.json'

def seed_pose():
    source=json.loads(SEED_RECORD.read_text(encoding='utf8'))
    return tuple(np.asarray(source[k],dtype=float) for k in ('targets','angles','gravity'))

class SitPlant(Plant):
    def __init__(self):
        super().__init__(XML,*seed_pose())
        self.butt_id=self.sim.model.geom('r5_cover_collision').id
        self.floor_id=self.sim.model.geom('sandbox_ground').id
        self.foot_ids=[self.sim.model.geom(n).id for n in ('left_foot_collision','right_foot_collision')]
        self.head_contact_ids=[i for i in range(self.sim.model.ngeom) if self.sim.model.geom_bodyid[i] in self.head_ids and self.sim.model.geom_group[i]==3]

    def ground_contacts(self):
        s=self.sim
        forces={'butt':0.,'feet':0.,'head':0.,'other':0.};contacts=[]
        for j in range(s.data.ncon):
            c=s.data.contact[j]
            if self.floor_id not in (c.geom1,c.geom2):continue
            g=c.geom2 if c.geom1==self.floor_id else c.geom1
            f=np.zeros(6);self.mj.mj_contactForce(s.model,s.data,j,f)
            kind='butt' if g==self.butt_id else 'feet' if g in self.foot_ids else 'head' if g in self.head_contact_ids else 'other'
            forces[kind]+=max(0.,float(f[0]));contacts.append({'geom':s.model.geom(g).name,'kind':kind,'force_n':float(f[0]),'z_mm':float(c.pos[2]*1000),'position_m':c.pos.tolist()})
        return forces,contacts

    def _record_substep(self):
        super()._record_substep()
        forces,_=self.ground_contacts()
        for key,value in forces.items():
            self.peak_contact_forces_n[key]=max(self.peak_contact_forces_n[key],value)
            self.contact_impulse_ns[key]+=value*.005

    def measure(self):
        s=self.sim;r=s.data.xmat[s.root_id].reshape(3,3);forces,contacts=self.ground_contacts()
        return {'t':float(s.data.time),'pitch_deg':math.degrees(math.atan2(r[0,2],r[2,2])),
                'roll_deg':math.degrees(math.atan2(r[2,1],r[2,2])),
                'head_clearance_mm':float(min(self.vertices(g)[:,2].min() for g in self.head_geoms)*1000),
                'butt_clearance_mm':float(self.vertices(self.butt_id)[:,2].min()*1000),
                'root_height_mm':float(s.data.qpos[2]*1000),'velocity':s.data.qvel[:6].tolist(),
                'forces_n':forces,'contacts':contacts,'qpos':s.data.qpos.tolist()}

def setup_renderer(plant,width=500,height=500):
    s=plant.sim;mj=plant.mj;s.model.vis.global_.offwidth=max(width,640);s.model.vis.global_.offheight=max(height,480)
    s.model.vis.headlight.active=1;s.model.vis.headlight.ambient[:]=[.65]*3;s.model.vis.headlight.diffuse[:]=[.8]*3
    for gid in range(s.model.ngeom):
        if s.model.geom_group[gid]==2 and not any(x in str(s.model.geom(gid).name) for x in ('uno','extension')):s.model.geom_rgba[gid]=[.42,.73,.81,1.]
    opt=mj.MjvOption();opt.geomgroup[3]=0
    renderer=mj.Renderer(s.model,height=height,width=width)
    cam=mj.MjvCamera();cam.azimuth=90;cam.elevation=-10;cam.distance=.48;cam.lookat[:]=[0,0,.11]
    return renderer,cam,opt

if __name__=='__main__':
    import matplotlib;matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.font_manager import FontProperties
    plant=SitPlant();s=plant.sim;mj=plant.mj;plant.reset(1000)
    pose=json.loads((ROOT/'pc/qa/post-calibration-20261025/butt-seat-goals.json').read_text())[0]
    renderer,cam,opt=setup_renderer(plant)
    qposes=[s.data.qpos.copy(),np.array(pose['qpos'])]
    font=FontProperties(fname='C:/Windows/Fonts/msyh.ttc');fig,axes=plt.subplots(1,2,figsize=(12,6),facecolor='#eef4f7')
    for ax,q,label in zip(axes,qposes,('已认可站姿','后壳接地终态候选 · 静态几何')):
        s.data.qpos[:]=q;mj.mj_forward(s.model,s.data);renderer.update_scene(s.data,camera=cam,scene_option=opt);ax.imshow(renderer.render());ax.axis('off');ax.set_title(label,fontproperties=font,fontsize=16)
    plt.tight_layout();out=ROOT/'pc/qa/post-calibration-20261025/butt-goal-preview.png';fig.savefig(out,dpi=140);plt.close(fig);renderer.close()
    print(json.dumps({'preview':str(out),'mouth_tip':s.data.site_xpos[s.model.site('mouth_tip').id].tolist(),
                     'head_center':s.data.xipos[s.model.body('jaw_soft').id].tolist(),'foot':s.data.site_xpos[s.model.site('left_foot').id].tolist(),
                     'goal':plant.measure()},ensure_ascii=False))
