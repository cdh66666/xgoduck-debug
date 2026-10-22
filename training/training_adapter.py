"""Create the weight-free native simulation; no hardware/network imports."""
import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'engine'))
import worker
class Adapter:
 def __init__(self,frozen,seed,scenario,enable_initial_policy=False):
  self.worker=worker
  class Counted(worker.Simulation):
   def __init__(self):self.reset_calls=0;super().__init__()
   def reset(self):self.reset_calls+=1;return super().reset()
   def step(self,count=1):
    self.max_substep_tilt_deg=self.max_substep_height_error_m=0.
    for _ in range(count):
     super().step(1)
     q=self.data.qpos[3:7];tilt=float(worker.np.degrees(worker.np.arccos(worker.np.clip(1-2*(q[1]**2+q[2]**2),-1.,1.))))
     self.max_substep_tilt_deg=max(self.max_substep_tilt_deg,tilt)
     self.max_substep_height_error_m=max(self.max_substep_height_error_m,abs(float(self.data.qpos[2])-self.standing_height))
  self.sim=Counted()
  bound={'sha256':'scratch','selected':'scratch','xml':str(ROOT/'model/robot.xml'),'bam':str(ROOT/'params/1910_m6.json'),
   'models':{n:{'path':None,'default_command':[0.,0.,0.]} for n in ('scratch','walk','getup')},'sources':{}}
  self.sim.load(bound,'scratch')
  if self.sim.extra_contacts!=13 or self.sim.model.opt.timestep!=.005:raise ValueError('physics contract changed')
  # Same initial yaw/joint perturbation contract as the frozen native evaluator.
  rng=worker.np.random.default_rng(seed);yaw=float(rng.uniform(-.03,.03));self.sim.data.qpos[3:7]=[worker.np.cos(yaw/2),0,0,worker.np.sin(yaw/2)]
  self.sim.data.qpos[self.sim.qadr]+=rng.uniform(-.005,.005,len(self.sim.qadr));self.sim.data.qvel.fill(0.)
  worker.mujoco.mj_forward(self.sim.model,self.sim.data)
  lows=[]
  for name in ('left_foot_collision','right_foot_collision'):
   gid=self.sim.model.geom(name).id;mid=self.sim.model.geom_dataid[gid];a,n=self.sim.model.mesh_vertadr[mid],self.sim.model.mesh_vertnum[mid]
   points=self.sim.model.mesh_vert[a:a+n]@self.sim.data.geom_xmat[gid].reshape(3,3).T+self.sim.data.geom_xpos[gid];lows.append(float(points[:,2].min()))
  self.sim.data.qpos[2]+=.001-min(lows);self.sim.standing_height=float(self.sim.data.qpos[2]);worker.mujoco.mj_forward(self.sim.model,self.sim.data)
 def close(self):pass
