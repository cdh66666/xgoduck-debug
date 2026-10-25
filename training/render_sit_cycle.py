"""Render an uncut, saved native sit/rise or multi-skill trajectory."""
import argparse,json,shutil,subprocess
from pathlib import Path
import numpy as np
from PIL import Image,ImageDraw,ImageFont
from butt_sit import SitPlant,setup_renderer

def main():
    p=argparse.ArgumentParser();p.add_argument('--trace',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--ffmpeg',default=shutil.which('ffmpeg'));p.add_argument('--font',default='C:/Windows/Fonts/msyh.ttc');a=p.parse_args()
    if not a.ffmpeg:p.error('Provide ffmpeg')
    rows=json.loads(a.trace.read_text());a.output.mkdir(parents=True,exist_ok=True)
    plant=SitPlant();renderer,cam,opt=setup_renderer(plant,960,600)
    cam.distance=.5;cam.elevation=-14;cam.azimuth=100
    plant.sim.model.geom_rgba[plant.floor_id]=[.10,.16,.20,1.]
    font=ImageFont.truetype(a.font,24);small=ImageFont.truetype(a.font,19)
    labels={'sit':'臀部坐下','raise':'起身','rise':'起身','stand':'站立','forward':'前进','stop':'停稳','stand_after':'起身后站立','backward':'后退','stop_final':'停止'}
    def frame(row):
        s=plant.sim;s.data.qpos[:]=row['qpos'];plant.mj.mj_forward(s.model,s.data)
        cam.lookat[:]=[row['qpos'][0],row['qpos'][1],.12]
        renderer.update_scene(s.data,camera=cam,scene_option=opt);scene=renderer.scene
        for axis in (0,1):
            for k in range(-30,31):
                start=np.array([-1.5,-1.5,.0005]);end=np.array([1.5,1.5,.0005]);start[axis]=end[axis]=k*.05
                geom=scene.geoms[scene.ngeom]
                plant.mj.mjv_initGeom(geom,plant.mj.mjtGeom.mjGEOM_LINE,np.zeros(3),np.zeros(3),np.eye(3).reshape(-1),np.array([.27,.4,.47,.7],np.float32))
                plant.mj.mjv_connector(geom,plant.mj.mjtGeom.mjGEOM_LINE,1.,start,end);scene.ngeom+=1
        for c in row['contacts']:
            if c['kind']!='butt' or c['force_n']<=.1:continue
            geom=scene.geoms[scene.ngeom];position=np.array(c['position_m']);position[2]=.002
            plant.mj.mjv_initGeom(geom,plant.mj.mjtGeom.mjGEOM_SPHERE,np.array([.004]*3),position,np.eye(3).reshape(-1),np.array([.15,1.,.45,1.],np.float32));scene.ngeom+=1
        image=Image.new('RGB',(960,720),'#0b1b28');image.paste(Image.fromarray(renderer.render()),(0,60));draw=ImageDraw.Draw(image)
        draw.text((24,16),'星猫 · '+labels.get(row['stage'],row['stage'])+' · 原生物理仿真',font=font,fill='#85e1f2')
        f=row['forces_n'];draw.rectangle((16,620,944,704),fill='#102b3a')
        draw.text((32,628),f"{row['t']:.2f} 秒    臀部 {f['butt']:.2f} N / 双脚 {f['feet']:.2f} N    头部离地 {row['head_clearance_mm']:.1f} mm",font=small,fill='white')
        draw.text((32,667),'50 Hz 推理 / 200 Hz 物理 · 全程无重置 · 未下发实机',font=small,fill='#b9d3df');return image
    process=subprocess.Popen([a.ffmpeg,'-hide_banner','-loglevel','error','-y','-f','rawvideo','-vcodec','rawvideo','-pix_fmt','rgb24','-s','960x720','-r','50','-i','-','-an','-c:v','libx264','-preset','fast','-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',str(a.output/'native-cycle.mp4')],stdin=subprocess.PIPE)
    for row in rows:process.stdin.write(frame(row).tobytes())
    process.stdin.close();assert process.wait()==0
    sit_rows=[r for r in rows if r['stage']=='sit'];rise_rows=[r for r in rows if r['stage'] in ('rise','raise')]
    selected=[rows[0],sit_rows[-1],rise_rows[min(35,len(rise_rows)-1)],rise_rows[min(130,len(rise_rows)-1)],rows[-1]]
    sheet=Image.new('RGB',(1440,792),'#0b1b28');draw=ImageDraw.Draw(sheet)
    for i,row in enumerate(selected):sheet.paste(frame(row).resize((480,360)),((i%3)*480,48+(i//3)*360))
    draw.text((24,9),'原始物理回放：站立 → 臀部坐地 → 重心前移 → 起身 → 站稳',font=font,fill='#85e1f2')
    for j,text in enumerate(['绿色点：臀部实际接触','原速 / 连续 / 无扶持','仅仿真，实机尚未验证']):draw.text((985,475+j*54),text,font=small,fill='#b9d3df')
    sheet.save(a.output/'five-frames.png');renderer.close()
    print(json.dumps({'frames':len(rows),'duration_s':len(rows)/50,'output':str(a.output)}))

if __name__=='__main__':main()
