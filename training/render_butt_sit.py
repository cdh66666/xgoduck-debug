"""Render actual saved native-physics motion, with contact metrics, no hardware."""
import argparse,hashlib,json,subprocess,shutil
from pathlib import Path
import numpy as np
from PIL import Image,ImageDraw,ImageFont
import onnxruntime as ort
from butt_sit import SitPlant,setup_renderer
from train_butt_sit import rollout

ROOT=Path(__file__).resolve().parents[1]
FONT='C:/Windows/Fonts/msyh.ttc'
def session_predict(file):
    options=ort.SessionOptions();options.intra_op_num_threads=1
    s=ort.InferenceSession(str(file),sess_options=options,providers=['CPUExecutionProvider'])
    return lambda x:s.run(None,{'observation':x.reshape(1,61)})[0][0]

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--ffmpeg',default=shutil.which('ffmpeg'),help='ffmpeg executable; defaults to PATH')
    ap.add_argument('--font',default=FONT,help='Chinese TrueType font path')
    ap.add_argument('--before-model',type=Path,help='Optional prior sit model for a comparison')
    args=ap.parse_args();out=args.output
    if not args.ffmpeg:ap.error('Install ffmpeg or provide --ffmpeg')
    evaluation=json.loads((out/'result.json').read_text())
    assert evaluation['onnx_sha256']==hashlib.sha256((out/'sit-butt-v3.onnx').read_bytes()).hexdigest()
    p=SitPlant()
    old=oldtrace=None
    if args.before_model:
        _,_,old,oldtrace=rollout(p,1000,session_predict(args.before_model),record=True,variant=False,duration=10,phase_duration=8.5)
    _,_,new,trace=rollout(p,1000,session_predict(out/'sit-butt-v3.onnx'),record=True,variant=False)
    (out/'comparison.json').write_text(json.dumps({'conditions':'same seed 1000, 7.4 V, friction 0.8, 200 Hz native BAM physics','v2':old,'v3':new},indent=2))
    (out/'render-trace.json').write_text(json.dumps(trace))
    renderer,cam,opt=setup_renderer(p,960,600);cam.distance=.47;cam.elevation=-14;cam.azimuth=100;cam.lookat[:]=[0,0,.12]
    p.sim.model.geom_rgba[p.floor_id]=[.10,.16,.20,1.]
    p.sim.model.vis.headlight.ambient[:]=[.4]*3;p.sim.model.vis.headlight.diffuse[:]=[.6]*3
    font=ImageFont.truetype(args.font,24);small=ImageFont.truetype(args.font,19)
    def frame(row,label):
        p.sim.data.qpos[:]=row['qpos'];p.mj.mj_forward(p.sim.model,p.sim.data)
        renderer.update_scene(p.sim.data,camera=cam,scene_option=opt)
        scene=renderer.scene
        for axis in (0,1):
            for k in range(-12,13):
                a=np.array([-.6,-.6,.0005]);b=np.array([.6,.6,.0005]);a[axis]=b[axis]=k*.05
                geom=scene.geoms[scene.ngeom]
                p.mj.mjv_initGeom(geom,p.mj.mjtGeom.mjGEOM_LINE,np.zeros(3),np.zeros(3),np.eye(3).reshape(-1),np.array([.27,.4,.47,.7],np.float32))
                p.mj.mjv_connector(geom,p.mj.mjtGeom.mjGEOM_LINE,1.,a,b);scene.ngeom+=1
        for contact in row['contacts']:
            if contact['kind']!='butt' or contact['force_n']<=.1:continue
            geom=scene.geoms[scene.ngeom];position=np.array(contact['position_m']);position[2]=.002
            p.mj.mjv_initGeom(geom,p.mj.mjtGeom.mjGEOM_SPHERE,np.array([.004]*3),position,np.eye(3).reshape(-1),np.array([.15,1.,.45,1.],np.float32));scene.ngeom+=1
        result=Image.new('RGB',(960,720),'#0b1b28');result.paste(Image.fromarray(renderer.render()),(0,60))
        draw=ImageDraw.Draw(result)
        draw.text((24,16),label,font=font,fill='#85e1f2')
        f=row['forces_n'];contact='臀部接地' if f['butt']>.5 else '臀部未接地'
        draw.rectangle((16,620,944,704),fill='#102b3a')
        draw.text((32,628),f"{row['t']:.2f} 秒   {contact}   臀部 {f['butt']:.2f} N / 双脚 {f['feet']:.2f} N",font=small,fill='white')
        draw.text((32,667),f"头部离地 {row['head_clearance_mm']:.1f} mm    仿真 50 Hz 推理 / 200 Hz 物理",font=small,fill='#b9d3df')
        return result
    process=subprocess.Popen([args.ffmpeg,'-hide_banner','-loglevel','error','-y','-f','rawvideo','-vcodec','rawvideo','-pix_fmt','rgb24','-s','960x720','-r','50','-i','-','-an','-c:v','libx264','-preset','fast','-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',str(out/'sit-butt-v3.mp4')],stdin=subprocess.PIPE)
    for row in trace:process.stdin.write(frame(row,'V3 · 一次落座并稳定支撑 · 仅物理仿真').tobytes())
    process.stdin.close();assert process.wait()==0
    times=[.02,.5,1.,1.2,5.];contact=Image.new('RGB',(1440,792),'#0b1b28');d=ImageDraw.Draw(contact)
    for i,t in enumerate(times):
        r=min(trace,key=lambda r:abs(r['t']-t));im=frame(r,'V3 · 物理仿真').resize((480,360));contact.paste(im,((i%3)*480,48+(i//3)*360))
    d.text((24,9),'从站立到臀部接地：5 帧原始物理回放',font=font,fill='#85e1f2')
    passed=sum(r['passed'] for r in evaluation['evaluations']);total=len(evaluation['evaluations'])
    d.text((990,470),f'{passed} / {total} 独立仿真通过',font=font,fill='#85e1f2')
    d.text((990,520),'绿色：臀部真实接触点',font=small,fill='#b9d3df')
    d.text((990,564),'约 1.1 秒落座，持续支撑',font=small,fill='#b9d3df')
    d.text((990,608),'仅物理仿真 · 未下发实机',font=small,fill='#b9d3df')
    contact.save(out/'five-frames.png')
    if oldtrace:
        comparison=Image.new('RGB',(1440,600),'#0b1b28')
        for i,(row,title) in enumerate([(oldtrace[-1],'V2 · 旧模型：低蹲'),(trace[-1],'V3 · 新模型：臀部落地')]):
            comparison.paste(frame(row,title).resize((720,540)),(720*i,0))
        draw=ImageDraw.Draw(comparison);draw.text((24,556),'同一物理模型 / 同一初始姿态 / 同一电压与摩擦系数；左为 10 秒末，右为 5 秒末。',font=small,fill='#b9d3df')
        comparison.save(out/'before-after.png')
    renderer.close()
    print(json.dumps({'old':old,'new':new,'video':str(out/'sit-butt-v3.mp4')},ensure_ascii=False))

if __name__=='__main__':main()
