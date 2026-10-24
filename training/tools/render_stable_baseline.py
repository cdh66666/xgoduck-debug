"""A continuous two-view recording of one native simulation, with no reset."""
import argparse,json,math,shutil,subprocess,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
from PIL import Image,ImageDraw,ImageFont
from stability_protocol import OnnxPolicy,new_episode,tick,duration,case_metrics
from render_native_video import make_renderer,frame


def main():
 p=argparse.ArgumentParser(description=__doc__)
 p.add_argument('--policy',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
 p.add_argument('--seed',type=int,default=20261420);a=p.parse_args()
 if a.output.exists():p.error('Keep previous recordings')
 a.output.parent.mkdir(parents=True,exist_ok=True)
 case=dict(title='连续站走停与向后受扰',segments=[(5.,[0.,0.,0.]),(5.,[.2,0.,0.]),(4.,[0.,0.,0.]),
  (5.,[-.3,0.,0.]),(5.,[0.,0.,0.]),(4.,[0.,0.,1.]),(4.,[0.,0.,0.]),
  (4.,[0.,0.,-1.]),(4.,[0.,0.,0.]),(8.,[0.,0.,0.])],
  pushes=[dict(t=43.,duration=.16,force=[-3.,0.,0.])])
 policy=OnnxPolicy(a.policy);env=new_episode(a.seed,max_episode_steps=round(duration(case)/.02)+1)
 views=[make_renderer(env,640,540) for _ in range(2)]
 views[0][1].azimuth=90;views[1][1].azimuth=0
 font='C:/Windows/Fonts/msyh.ttc';fonts=[ImageFont.truetype(font,s) for s in (25,22,20)]
 command=[shutil.which('ffmpeg'),'-hide_banner','-loglevel','warning','-f','rawvideo','-pix_fmt','rgb24',
  '-s','1280x696','-r','25','-i','-','-an','-c:v','libx264','-preset','fast','-crf','21',
  '-threads','2','-pix_fmt','yuv420p','-movflags','+faststart',str(a.output)]
 rows=[];samples=[];count=0;max_substep_tilt=0.
 selected={round(t/.02)-1 for t in (4.,18.,19.2,43.08,46.)}
 with a.output.with_suffix('.ffmpeg.log').open('w',encoding='utf-8') as log:
  encoder=subprocess.Popen(command,stdin=subprocess.PIPE,stdout=subprocess.DEVNULL,stderr=log)
  try:
   for index in range(round(duration(case)/.02)):
    row,done=tick(env,policy,case,index)
    rotation=env.sim.data.xmat[env.sim.root_id].reshape(3,3)
    row['backward_pitch_deg']=math.degrees(math.atan2(rotation[2,0],math.hypot(rotation[0,0],rotation[1,0])))
    max_substep_tilt=max(max_substep_tilt,float(env.sim.max_substep_tilt_deg));rows.append(row)
    if index%2==1:
     canvas=Image.new('RGB',(1280,696),(15,22,32));draw=ImageDraw.Draw(canvas)
     draw.text((18,8),'站立基线 · MuJoCo 原生物理仿真 · 1×速度 · 连续 48 秒无重置',font=fonts[0],fill=(235,240,247))
     for side,view in enumerate(views):
      x=640*side;canvas.paste(Image.fromarray(frame(*view,env)),(x,82))
      draw.text((x+18,46),'侧面' if side==0 else '正面',font=fonts[1],fill=(108,195,245))
     cmd=row['command'];label='站立 / 停止' if not any(cmd) else ('前进 0.2 m/s' if cmd[0]>0 else '后退 0.3 m/s' if cmd[0]<0 else '原地转向')
     draw.text((18,628),f"{row['t']:05.2f}s  |  {label}  |  后仰 {row['backward_pitch_deg']:+.2f}°",font=fonts[1],fill=(235,240,247))
     draw.text((18,660),'43.0s：向后施加 3N 推力，持续 0.16s；全片仅一次初始化。非实机。',font=fonts[2],fill=(210,218,230))
     if 43<=row['t']<=44.2:draw.text((20,104),'向后受扰 → 自主恢复站立',font=fonts[1],fill=(255,203,111))
     if row['failure']:draw.text((20,140),'FAIL '+','.join(row['failure']),font=fonts[1],fill=(255,80,80))
     encoder.stdin.write(np.asarray(canvas).tobytes());count+=1
     if count==1:canvas.save(a.output.with_suffix('.preview.png'))
     if index in selected:
      path=a.output.with_name(a.output.stem+f'-{row["t"]:.2f}s.png');canvas.save(path);samples.append((row['t'],canvas.copy(),str(path)))
    if done:break
  finally:
   encoder.stdin.close();code=encoder.wait()
   for renderer,_,_ in views:renderer.close()
   env.close()
 if code:raise RuntimeError('ffmpeg failed')
 sheet=Image.new('RGB',(1280,348*3),(15,22,32))
 for i,(_,sample,_) in enumerate(samples):sheet.paste(sample.resize((640,348)),((i%2)*640,(i//2)*348))
 sheet.save(a.output.with_suffix('.five-frames.png'))
 metrics=case_metrics(rows,case)
 result=dict(policy=str(a.policy.resolve()),sha256=policy.sha256,seed=a.seed,hardware_operated=False,
  seconds=count/25.,fps=25,frames=count,continuous=True,metrics=metrics,max_substep_tilt_deg=max_substep_tilt,
  max_backward_pitch_deg=max(r['backward_pitch_deg'] for r in rows),
  resets_during_recording=max(r['resets'] for r in rows)-min(r['resets'] for r in rows),
  sampled_frames=[dict(t=t,path=path) for t,_,path in samples],case=case,rows=rows)
 a.output.with_suffix('.json').write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
 print(json.dumps({k:v for k,v in result.items() if k not in ('rows','case','sampled_frames')},ensure_ascii=False),flush=True)


if __name__=='__main__':main()
