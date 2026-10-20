/* Read-only communication colors on the real q=0 meshes. The model never moves. */
export const SERVO_IDS=Object.freeze([10,11,12,13,14,20,21,22,23,24,30,31,32,33,34]);
export const LINK_IDS=Object.freeze({left_hip_yaw:10,left_hip_roll:11,left_hip_pitch:12,left_knee:13,left_ankle:14,
  right_hip_yaw:20,right_hip_roll:21,right_hip_pitch:22,right_knee:23,right_ankle:24,neck_pitch:30,head_pitch:31,head_yaw:32,head_roll:33});
const legalIds=ids=>Array.isArray(ids)&&ids.length<=15&&ids.every(id=>Number.isInteger(id)&&SERVO_IDS.includes(id))&&new Set(ids).size===ids.length;
export function connectionColors(status){
  const known=!!status&&legalIds(status.servo_ids)&&typeof status.imu_ok==='boolean',answered=new Set(known?status.servo_ids:[]);
  const ids=Object.fromEntries(SERVO_IDS.map(id=>[id,known?(answered.has(id)?'online':'missing'):'unknown']));
  return {ids,links:Object.fromEntries(Object.entries(LINK_IDS).map(([name,id])=>[name,ids[id]])),mouth:ids[34],imu:known?(status.imu_ok?'online':'missing'):'unknown',count:known?answered.size:null};
}
export class ConnectionTracker {
  constructor(){this.sequence=null;this.instance=null;this.lastNew=null;this.status=null;this.mode='waiting';this.reason='等待反馈';}
  hold(mode,reason){this.mode=mode;this.reason=reason;this.status=null;return {changed:true,mode,reason,colors:connectionColors(null)};}
  consume(message,now,wallNow=Date.now()){
    if(message?.type!=='connection-model-state'||message.version!==1||!['live','stale','paused'].includes(message.mode))return null;
    if(message.mode!=='live'){
      const reasons=['离线','页面已暂停','读取暂停','反馈过期','反馈未前进','等待反馈','反馈无效'];
      return this.hold(message.mode,reasons.includes(message.reason)?message.reason:'反馈暂停');
    }
    const input=message.status,s=input?{mcu_seq:input.mcu_seq,received:input.received,feedback_age_ms:input.feedback_age_ms,
      app_instance_id:input.app_instance_id,servo_ids:Array.isArray(input.servo_ids)?input.servo_ids.slice():null,imu_ok:input.imu_ok}:null;
    if(!s||!Number.isInteger(s.mcu_seq)||s.mcu_seq<0||s.mcu_seq>0xffffffff||!Number.isFinite(s.received)||s.received<=0||
      !Number.isFinite(s.feedback_age_ms)||s.feedback_age_ms<0||s.feedback_age_ms>=150||typeof s.app_instance_id!=='string'||!s.app_instance_id||!legalIds(s.servo_ids)||typeof s.imu_ok!=='boolean')return this.hold('stale','反馈无效');
    if(!Number.isFinite(message.observedAtMs)||message.observedAtMs<=0)return this.hold('stale','采样时间未知');
    const cachedAge=wallNow-message.observedAtMs;
    if(cachedAge<0||cachedAge+s.feedback_age_ms>1500)return this.hold('stale','反馈过期');
    const delta=this.sequence===null?1:(s.mcu_seq-this.sequence)>>>0;
    if(this.sequence!==null&&(this.instance!==s.app_instance_id||delta>=0x80000000)){
      this.sequence=s.mcu_seq;this.instance=s.app_instance_id;this.lastNew=null;return this.hold('stale','重启后等待新反馈');
    }
    if(this.sequence!==null&&delta===0){
      if(this.lastNew===null||now-this.lastNew>1500)return this.hold('stale','反馈未前进');
      return {changed:false,mode:this.mode,reason:this.reason,colors:connectionColors(this.status)};
    }
    this.sequence=s.mcu_seq;this.instance=s.app_instance_id;this.lastNew=now-cachedAge-s.feedback_age_ms;this.status=s;this.mode='live';this.reason='实时通信';
    return {changed:true,mode:this.mode,reason:this.reason,colors:connectionColors(s)};
  }
  tick(now){return this.mode==='live'&&this.lastNew!==null&&now-this.lastNew>1500?this.hold('stale','反馈过期'):null;}
}

async function boot(){
  const get=id=>document.getElementById(id),tracker=new ConnectionTracker(),meshes=new Map(),markers=new Map(),labels=new Map(),chips=new Map();
  let pending=null,loaded=false,colors=connectionColors(null),THREE,scene,camera,renderer,controls,model,center,size,mouthPoint,imuPoint;
  const viewport=get('connection-model-viewport');
  for(const id of SERVO_IDS){const chip=document.createElement('span');chip.className='id-chip';chip.dataset.state='unknown';chip.textContent=String(id);chip.title='ID '+id+'：未知';get('connection-model-ids').append(chip);chips.set(id,chip);}
  const stateText=state=>state==='online'?'已应答':state==='missing'?'未应答':'未知';
  const palette={unknown:{color:0x526675,emissive:0x000000,intensity:0},online:{color:0x6eb7c6,emissive:0x0d4d60,intensity:.14},missing:{color:0xff1745,emissive:0xf5093e,intensity:.72}};
  function paint(){
    for(const [name,mesh] of meshes){const state=colors.links[name]||'unknown',p=palette[state];mesh.material.color.setHex(p.color);mesh.material.emissive.setHex(p.emissive);mesh.material.emissiveIntensity=p.intensity;mesh.userData.connectionState=state;}
    for(const [id,marker] of markers){marker.material.color.setHex(palette[colors.ids[id]].color);marker.material.opacity=colors.ids[id]==='unknown'?.30:1;labels.get(id).hidden=colors.ids[id]!=='missing';}
    for(const [id,chip] of chips){chip.dataset.state=colors.ids[id];chip.title='ID '+id+'：'+stateText(colors.ids[id]);}
    for(const [id,state] of [['connection-mouth',colors.mouth],['connection-imu',colors.imu]]){const el=get(id);el.dataset.state=state;el.querySelector('strong').textContent=stateText(state);}
    get('connection-mouth-leader').dataset.state=colors.mouth;get('connection-imu-leader').dataset.state=colors.imu;
  }
  let paintKey=null;
  const apply=result=>{
    if(!result)return;
    const key=JSON.stringify([result.mode,result.reason,result.colors]);if(key===paintKey)return;paintKey=key;
    colors=result.colors;document.body.dataset.mode=result.mode;document.body.dataset.live=String(result.mode==='live');
    get('connection-model-state').textContent=result.mode==='live'?'应答 '+colors.count+'/15 · '+result.reason:result.reason;
    paint();
  };
  window.addEventListener('message',event=>{
    if(event.origin!==location.origin||event.source!==window.parent||event.data?.type!=='connection-model-state')return;
    if(document.hidden){pending=null;apply(tracker.hold('paused','页面已暂停'));return;}
    if(!loaded){pending=event.data;return;}apply(tracker.consume(event.data,performance.now()));
  });
  document.addEventListener('visibilitychange',()=>{if(document.hidden){pending=null;apply(tracker.hold('paused','页面已暂停'));}});
  setInterval(()=>apply(tracker.tick(performance.now())),100);
  try{
    const modules=await Promise.all([import('../model-assets/vendor/build/three.module.js'),import('../model-assets/vendor/examples/jsm/controls/OrbitControls.js'),import('../model-assets/vendor/examples/jsm/loaders/STLLoader.js')]);
    THREE=modules[0];const {OrbitControls}=modules[1],{STLLoader}=modules[2];
    scene=new THREE.Scene();scene.add(new THREE.HemisphereLight(0xc9f2ff,0x16212c,2.6));
    const key=new THREE.DirectionalLight(0xdff8ff,3.1);key.position.set(.35,-.45,.45);scene.add(key);
    const rim=new THREE.DirectionalLight(0x46c8e0,2);rim.position.set(-.3,.25,.25);scene.add(rim);
    camera=new THREE.PerspectiveCamera(34,1,.001,10);camera.up.set(0,0,1);
    renderer=new THREE.WebGLRenderer({antialias:true,alpha:true});renderer.setPixelRatio(Math.min(devicePixelRatio,1.5));renderer.setClearColor(0x071621,0);renderer.toneMapping=THREE.ACESFilmicToneMapping;renderer.toneMappingExposure=1.05;viewport.append(renderer.domElement);
    controls=new OrbitControls(camera,renderer.domElement);controls.enableDamping=true;controls.enablePan=false;controls.minDistance=.20;controls.maxDistance=.9;
    const response=await fetch('../model-assets/model-data.json');if(!response.ok)throw Error('模型清单未读取');
    const data=await response.json(),pose=data.poses?.['q-zero'];
    if(data.records?.length!==15||!pose?.records?.trunk_base||data.independent_mouth_axis_available!==false||Object.keys(pose.joints||{}).length!==14)throw Error('模型基线不完整');
    for(const [name,id] of Object.entries(LINK_IDS))if(!data.joints.some(j=>j.id===id&&j.name===name&&Array.isArray(j.axis)))throw Error('关节编号不一致');
    model=new THREE.Group();scene.add(model);
    const pivot=new THREE.Vector3(...pose.records.trunk_base.slice(0,3).map(row=>row[3])),shift=new THREE.Matrix4().makeTranslation(-pivot.x,-pivot.y,-pivot.z),loader=new STLLoader();
    await Promise.all(data.records.map(async record=>{
      const geometry=await loader.loadAsync('../model-assets/'+record.name+'.stl');geometry.computeVertexNormals();
      const material=new THREE.MeshStandardMaterial({color:palette.unknown.color,roughness:.42,metalness:.30});
      const mesh=new THREE.Mesh(geometry,material);mesh.name=record.name;mesh.matrixAutoUpdate=false;mesh.matrix.set(...pose.records[record.name].flat());mesh.matrix.premultiply(shift);model.add(mesh);meshes.set(record.name,mesh);
    }));
    scene.updateMatrixWorld(true);const box=new THREE.Box3().setFromObject(model);center=box.getCenter(new THREE.Vector3());size=box.getSize(new THREE.Vector3());
    // These two region anchors are bounding-box centers, never invented sensor/mouth axes.
    mouthPoint=new THREE.Box3().setFromObject(meshes.get('head_roll')).getCenter(new THREE.Vector3());
    imuPoint=new THREE.Box3().setFromObject(meshes.get('trunk_base')).getCenter(new THREE.Vector3());
    for(const [idString,joint] of Object.entries(pose.joints)){
      const id=Number(idString),position=new THREE.Vector3(...joint.origin_m).sub(pivot);
      const marker=new THREE.Mesh(new THREE.SphereGeometry(.0028,12,8),new THREE.MeshBasicMaterial({color:palette.unknown.color,transparent:true,opacity:.30,depthTest:false}));marker.position.copy(position);marker.renderOrder=3;scene.add(marker);markers.set(id,marker);
      const label=document.createElement('span');label.className='joint-tag';label.textContent=String(id);label.hidden=true;viewport.append(label);labels.set(id,label);
    }
    const grid=new THREE.GridHelper(.38,12,0x3b7185,0x183647);grid.rotation.x=Math.PI/2;grid.position.z=box.min.z-.018;grid.material.transparent=true;grid.material.opacity=.26;scene.add(grid);
    function home(){const span=Math.max(size.x,size.y,size.z);camera.position.copy(center).add(new THREE.Vector3(span*1.15,-span*1.65,span*.95));controls.target.copy(center);controls.update();}
    function resize(){const bounds=viewport.getBoundingClientRect();if(bounds.width<2||bounds.height<2){apply(tracker.hold('paused','页面已暂停'));return;}camera.aspect=bounds.width/bounds.height;camera.updateProjectionMatrix();renderer.setSize(bounds.width,bounds.height);}
    function screenPoint(point){const p=point.clone().project(camera);return {x:(p.x+1)*viewport.clientWidth/2,y:(1-p.y)*viewport.clientHeight/2,visible:p.z>=-1&&p.z<=1&&Math.abs(p.x)<=1&&Math.abs(p.y)<=1};}
    function leader(name,point){const el=get('connection-'+name),line=get('connection-'+name+'-leader'),p=screenPoint(point),r=el.getBoundingClientRect(),v=viewport.getBoundingClientRect();line.setAttribute('x1',p.x);line.setAttribute('y1',p.y);line.setAttribute('x2',r.left+r.width/2-v.left);line.setAttribute('y2',r.top+r.height/2-v.top);line.style.display=p.visible?'':'none';}
    get('connection-model-camera').addEventListener('click',home);new ResizeObserver(resize).observe(viewport);home();resize();
    document.body.dataset.loaded='true';document.body.dataset.meshCount=String(model.children.length);document.body.dataset.modelVersion=data.version;document.body.dataset.referencePose='q-zero';
    loaded=true;if(document.hidden){pending=null;apply(tracker.hold('paused','页面已暂停'));}else{apply(tracker.hold('waiting','等待反馈'));if(pending){apply(tracker.consume(pending,performance.now()));pending=null;}}
    function render(){requestAnimationFrame(render);if(document.hidden||tracker.mode==='paused'||viewport.clientWidth<2||viewport.clientHeight<2)return;controls.update();camera.updateMatrixWorld();renderer.render(scene,camera);
      for(const [id,marker] of markers){const label=labels.get(id),p=screenPoint(marker.position);label.hidden=colors.ids[id]!=='missing'||!p.visible;label.style.left=p.x+'px';label.style.top=(p.y-6)+'px';}
      leader('mouth',mouthPoint);leader('imu',imuPoint);
    }requestAnimationFrame(render);
    if(window.parent!==window)window.parent.postMessage({type:'connection-model-ready',version:1,meshes:model.children.length,modelVersion:data.version},location.origin);
  }catch(error){document.body.dataset.loaded='false';apply(tracker.hold('stale','模型未加载'));get('connection-model-state').title=String(error.message);}
}
if(typeof document!=='undefined'&&typeof window!=='undefined')boot();
