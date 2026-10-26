"""Render an uncut, saved native sit/rise or multi-skill trajectory."""
import argparse,json,shutil,subprocess
from pathlib import Path
import numpy as np
from PIL import Image,ImageDraw,ImageFont
from butt_sit import SitPlant,setup_renderer

def main():
    p=argparse.ArgumentParser();p.add_argument('--trace',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--ffmpeg',default=shutil.which('ffmpeg'));p.add_argument('--font',default='C:/Windows/Fonts/msyh.ttc')
    p.add_argument('--azimuth',type=float,default=100)
    p.add_argument('--sit-style',choices=('rear-cover','official-reference'),default='rear-cover')
    p.add_argument('--controller-kind',choices=('policy','teacher'),default='policy');a=p.parse_args()
    if not a.ffmpeg:p.error('Provide ffmpeg')
    rows=json.loads(a.trace.read_text());a.output.mkdir(parents=True,exist_ok=True)
    plant=SitPlant();renderer,cam,opt=setup_renderer(plant,960,600)
    cam.distance=.5;cam.elevation=-14;cam.azimuth=a.azimuth
    plant.sim.model.geom_rgba[plant.floor_id]=[.10,.16,.20,1.]
    font=ImageFont.truetype(a.font,24);small=ImageFont.truetype(a.font,19)
    labels={'sit':'臀部坐下','raise':'起身','rise':'起身','stand':'站立','forward':'前进','stop':'停稳','stand_after':'起身后站立','backward':'后退','stop_final':'停止','dance':'摆身舞蹈','single_support':'单脚支撑候选','roll':'前滚翻候选'}
    seat_geoms={'r5_cover_collision'}
    if a.sit_style=='official-reference':
        labels['sit']='官方参考坐姿候选'
        seat_geoms.update(('sandbox_contact_left_hip_pitch','sandbox_contact_right_hip_pitch'))
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
            if c['geom'] not in seat_geoms or c['force_n']<=.1:continue
            geom=scene.geoms[scene.ngeom];position=np.array(c['position_m']);position[2]=.002
            plant.mj.mjv_initGeom(geom,plant.mj.mjtGeom.mjGEOM_SPHERE,np.array([.004]*3),position,np.eye(3).reshape(-1),np.array([.15,1.,.45,1.],np.float32));scene.ngeom+=1
        image=Image.new('RGB',(960,720),'#0b1b28');image.paste(Image.fromarray(renderer.render()),(0,60));draw=ImageDraw.Draw(image)
        draw.text((24,16),'星猫 · '+labels.get(row['stage'],row['stage'])+' · 原生物理仿真',font=font,fill='#85e1f2')
        f=row['forces_n'];draw.rectangle((16,620,944,704),fill='#102b3a')
        contact=f"头部 {f['head']:.2f} N" if row['stage']=='roll' else f"臀部 {f['butt']:.2f} N"
        if a.sit_style=='official-reference' and row['stage']!='roll':
            support=sum(c['force_n'] for c in row['contacts'] if c['geom'] in seat_geoms)
            contact=f"腿壳/后盖支撑 {support:.2f} N"
        draw.text((32,628),f"{row['t']:.2f} 秒    {contact} / 双脚 {f['feet']:.2f} N    头部离地 {row['head_clearance_mm']:.1f} mm",font=small,fill='white')
        control='推理' if a.controller_kind=='policy' else '教师控制（尚非神经模型）'
        draw.text((32,667),f'50 Hz {control} / 200 Hz 物理 · 全程无重置 · 未下发实机',font=small,fill='#b9d3df');return image
    process=subprocess.Popen([a.ffmpeg,'-hide_banner','-loglevel','error','-y','-f','rawvideo','-vcodec','rawvideo','-pix_fmt','rgb24','-s','960x720','-r','50','-i','-','-an','-c:v','libx264','-preset','fast','-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',str(a.output/'native-cycle.mp4')],stdin=subprocess.PIPE)
    for row in rows:process.stdin.write(frame(row).tobytes())
    process.stdin.close();assert process.wait()==0
    sit_rows=[r for r in rows if r['stage']=='sit'];rise_rows=[r for r in rows if r['stage'] in ('rise','raise')]
    forward=[r for r in rows if r['stage']=='forward'];backward=[r for r in rows if r['stage']=='backward']
    roll_rows=[r for r in rows if r['stage']=='roll']
    if roll_rows:
        # Uniform samples over roll+hold can miss the entire one-second roll.
        # Sample actual accumulated pitch, retaining start and final hold.
        angle=np.rad2deg(np.unwrap(np.deg2rad([r['pitch_deg'] for r in roll_rows])))
        selected=[rows[0],*[roll_rows[int(np.argmin(np.abs(angle-target)))] for target in (90,180,270)],rows[-1]]
    elif forward and backward and sit_rows and rise_rows:
        # A multi-skill contact sheet must also show the walking phases; a
        # sit-only selection would omit the very motion under investigation.
        selected=[rows[0],forward[len(forward)//2],sit_rows[-1],rise_rows[min(130,len(rise_rows)-1)],backward[len(backward)//2]]
    elif forward and sit_rows and not rise_rows:
        # Preserve the failing phase in partial chains; uniform sampling can
        # spend four of five frames on the successful walking prefix.
        stopped=[r for r in rows if r['stage']=='stop']
        selected=[rows[0],forward[len(forward)//2],stopped[-1] if stopped else sit_rows[0],sit_rows[len(sit_rows)//2],rows[-1]]
    else:selected=[rows[0],sit_rows[-1],rise_rows[min(35,len(rise_rows)-1)],rise_rows[min(130,len(rise_rows)-1)],rows[-1]] if sit_rows and rise_rows else [rows[round(t*(len(rows)-1))] for t in (0,.22,.48,.74,1)]
    sheet=Image.new('RGB',(1440,792),'#0b1b28');draw=ImageDraw.Draw(sheet)
    for i,row in enumerate(selected):sheet.paste(frame(row).resize((480,360)),((i%3)*480,48+(i//3)*360))
    heading='站立 → 臀部坐地 → 重心前移 → 起身 → 站稳' if sit_rows and rise_rows else labels.get(rows[0]['stage'],rows[0]['stage'])+' · 五帧连续物理证据'
    if a.sit_style=='official-reference' and sit_rows and rise_rows:heading='站立 → 官方参考坐姿 → 重心转移 → 起身 → 站稳（候选）'
    if forward and backward and sit_rows and rise_rows:heading='站立 → 前进 → 坐地 → 起身 → 后退（完整视频含起停）'
    if forward and sit_rows and not rise_rows:heading='站立 → 前进 → 停止 → 坐下（记录在此结束，未完成起身）'
    if roll_rows:heading='起始 → 约90° → 倒置 → 约270° → 站稳'
    draw.text((24,9),'原始物理回放：'+heading,font=font,fill='#85e1f2')
    evidence='双脚支撑 · 基础摆身舞蹈' if rows[0]['stage']=='dance' else '绿色点：臀部实际接触'
    if a.sit_style=='official-reference':evidence='绿色点：腿壳/后盖接触'
    if roll_rows:evidence='头部接触地面，实机强度未验证'
    for j,text in enumerate([evidence,'原速 / 连续 / 无扶持','仅仿真，实机尚未验证']):draw.text((985,475+j*54),text,font=small,fill='#b9d3df')
    sheet.save(a.output/'five-frames.png');renderer.close()
    print(json.dumps({'frames':len(rows),'duration_s':len(rows)/50,'output':str(a.output)}))

if __name__=='__main__':main()
