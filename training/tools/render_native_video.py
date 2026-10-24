"""Render two real native MuJoCo rollouts side by side, without reset in a clip."""
import argparse,gc,json,math,shutil,subprocess,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
import mujoco
from PIL import Image,ImageDraw,ImageFont
from stability_protocol import OnnxPolicy,scenarios,duration,new_episode,tick,case_metrics


def make_renderer(env,width,height):
 model=env.sim.model
 model.vis.global_.offwidth=max(width,model.vis.global_.offwidth)
 model.vis.global_.offheight=max(height,model.vis.global_.offheight)
 model.vis.headlight.ambient[:]=[.5,.5,.5];model.vis.headlight.diffuse[:]=[.65,.65,.65]
 model.geom_rgba[model.geom('sandbox_ground').id]=[.14,.18,.23,1.]
 for i in range(model.ngeom):
  if model.geom_group[i]==2:model.geom_rgba[i]=[.75,.85,.91,1.]
 option=mujoco.MjvOption();option.geomgroup[:]=0;option.geomgroup[0]=1;option.geomgroup[2]=1
 camera=mujoco.MjvCamera();camera.distance=.55;camera.azimuth=45;camera.elevation=-20
 return mujoco.Renderer(model,height=height,width=width),camera,option


def frame(renderer,camera,option,env):
 root=env.sim.data.xpos[env.sim.root_id]
 camera.lookat[:]=root+[0,0,.015]
 renderer.update_scene(env.sim.data,camera=camera,scene_option=option)
 scene=renderer.scene
 for axis in (0,1):
  center=float(root[axis]);lo=math.floor((center-.6)/.05);hi=math.ceil((center+.6)/.05)
  for k in range(lo,hi+1):
   a=np.array([root[0]-.65,root[1]-.65,.001]);b=np.array([root[0]+.65,root[1]+.65,.001])
   a[axis]=b[axis]=k*.05
   rgba=np.array([.33,.4,.48,.55],dtype=np.float32)
   geom=scene.geoms[scene.ngeom]
   mujoco.mjv_initGeom(geom,mujoco.mjtGeom.mjGEOM_LINE,np.zeros(3),np.zeros(3),np.eye(3).reshape(-1),rgba)
   mujoco.mjv_connector(geom,mujoco.mjtGeom.mjGEOM_LINE,1.,a,b);scene.ngeom+=1
 return renderer.render()


def main():
 p=argparse.ArgumentParser(description=__doc__)
 p.add_argument('--left',type=Path,required=True);p.add_argument('--right',type=Path,required=True)
 p.add_argument('--left-label',default='原版 47500');p.add_argument('--right-label',default='上一轮候选（尚未证明更稳）')
 p.add_argument('--scenarios',default='stand4,forward_stop,backward_stop,left_stop,right_stop')
 p.add_argument('--seed',type=int,default=20261030);p.add_argument('--push-n',type=float,default=.8)
 p.add_argument('--width',type=int,default=640);p.add_argument('--height',type=int,default=540)
 p.add_argument('--output',type=Path,required=True);args=p.parse_args()
 if args.output.exists():p.error('Use a new video path')
 args.output.parent.mkdir(parents=True,exist_ok=True)
 ffmpeg=shutil.which('ffmpeg')
 if not ffmpeg:raise RuntimeError('ffmpeg is not installed')
 policies=[OnnxPolicy(args.left),OnnxPolicy(args.right)]
 font_path='C:/Windows/Fonts/msyh.ttc'
 fonts=[ImageFont.truetype(font_path,s) for s in (26,23,20)]
 width,height=args.width*2,args.height+156
 command=[ffmpeg,'-hide_banner','-loglevel','warning','-f','rawvideo','-pix_fmt','rgb24','-s',f'{width}x{height}',
  '-r','25','-i','-','-an','-c:v','libx264','-preset','fast','-crf','21','-threads','2','-pix_fmt','yuv420p','-movflags','+faststart',str(args.output)]
 reports=[];count=0
 with args.output.with_suffix('.ffmpeg.log').open('w',encoding='utf-8') as log:
  encoder=subprocess.Popen(command,stdin=subprocess.PIPE,stdout=subprocess.DEVNULL,stderr=log)
  try:
   for key in args.scenarios.split(','):
    case=scenarios(args.push_n)[key];envs=[new_episode(args.seed) for _ in policies]
    views=[make_renderer(e,args.width,args.height) for e in envs];rows=[[],[]];done=[False,False]
    try:
     for index in range(round(duration(case)/.02)):
      for side,(env,policy) in enumerate(zip(envs,policies)):
       if not done[side]:
        row,done[side]=tick(env,policy,case,index);rows[side].append(row)
      if index%2!=1:continue
      canvas=Image.new('RGB',(width,height),(15,22,32));draw=ImageDraw.Draw(canvas)
      draw.text((22,8),'MuJoCo 原生物理仿真 · 1×速度 · 非实机 · 每段片内不重置',font=fonts[0],fill=(235,240,247))
      for side,env in enumerate(envs):
       x=side*args.width;renderer,camera,option=views[side]
       canvas.paste(Image.fromarray(frame(renderer,camera,option,env)),(x,82))
       color=(78,170,246) if side==0 else (255,164,76)
       draw.text((x+18,46),args.left_label if side==0 else args.right_label,font=fonts[1],fill=color)
       r=rows[side][-1];v=r['velocity'] or [0.,0.,0.];cmd=r['command']
       state='停止指令' if not any(cmd) else '运动指令'
       draw.text((x+18,args.height+88),f"{case['title']} | {r['t']:.2f}s | {state}",font=fonts[2],fill=(235,240,247))
       draw.text((x+18,args.height+118),f"vx {v[0]:+.3f} m/s   yaw {v[2]:+.2f} rad/s   倾角 {r['tilt'] or 0:.2f}°",font=fonts[2],fill=color)
       if case.get('pushes'):
        draw.text((x+18,104),f"3.0s 施加 {args.push_n:g} N 推力，持续 0.16s",font=fonts[2],fill=(240,218,143))
       if r['failure']:draw.text((x+20,104),'FAIL: '+','.join(r['failure']),font=fonts[1],fill=(255,75,75))
      encoder.stdin.write(np.asarray(canvas).tobytes());count+=1
      if count==1:canvas.save(args.output.with_suffix('.preview.png'))
     reports.append({'scenario':key,'seed':args.seed,'left':case_metrics(rows[0],case),'right':case_metrics(rows[1],case),'rows':rows})
     print(json.dumps({'scenario':key,'frames_total':count,'left_failure':reports[-1]['left']['failure'],'right_failure':reports[-1]['right']['failure']}),flush=True)
    finally:
     for renderer,_,_ in views:renderer.close()
     for env in envs:env.close()
     del views,envs;gc.collect()
  finally:
   encoder.stdin.close();code=encoder.wait()
  if code:raise RuntimeError('ffmpeg failed; see '+str(args.output.with_suffix('.ffmpeg.log')))
 result={'scope':'Native MuJoCo 3.15.0 / BAM. New independent scene between clips; no reset inside a clip. Not hardware.',
  'left_sha256':policies[0].sha256,'right_sha256':policies[1].sha256,'fps':25,'frames':count,'seconds':count/25.,'cases':reports}
 args.output.with_suffix('.json').write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False),'utf-8')
 print(json.dumps({'video':str(args.output),'seconds':count/25.,'frames':count}),flush=True)


if __name__=='__main__':main()
