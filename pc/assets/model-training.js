/* Live MuJoCo poses only. These APIs cannot address the physical robot. */
import * as THREE from 'three';
import { OrbitControls } from '/model-replay-assets/vendor/examples/jsm/controls/OrbitControls.js';
import { STLLoader } from '/model-replay-assets/vendor/examples/jsm/loaders/STLLoader.js';
import { OBJLoader } from '/model-replay-assets/vendor/examples/jsm/loaders/OBJLoader.js';

const $=id=>document.getElementById(id),host=$('viewport');
const modelId=new URLSearchParams(location.search).get('model');
const externalControls=new URLSearchParams(location.search).get('controls')==='external';
let renderer,scene,camera,controls,session,token,events,state,resizeObserver,initDone,jawPivot,jawAxis;
let visible=true,disposed=false,drag=null,lastGrabAt=0,grabPending=false,grabTimer=null,errorMessage='',requestChain=Promise.resolve();
const virtualMotion=modelId==='official-walk',heldInputs=new Map(),keyboard=window.DuckKeyboard.create();
let lastTwist='',desiredTwist=null,motionSending=null,functionBusy=false,followPosition=null;
const objects=[],pickables=[],raycaster=new THREE.Raycaster(),pointer=new THREE.Vector2();
const arrow=new THREE.ArrowHelper(new THREE.Vector3(1,0,0),new THREE.Vector3(),1,0x60d9f0,16,9);
arrow.visible=false;

function modelColor(record){
  // Geometry ids are the same mesh names used by the live attitude renderer;
  // physics body names differ (for example neck_pitch contains head_pitch).
  const name=record.id;
  if(name==='uno-q'||name==='extension'||name==='cover')return record.color;
  return name==='mouth_jaw'?0xd4a454:name.includes('ankle')?0xb69467:name.includes('head')?0xa1bac8:name==='trunk_base'?0x425d6b:0x698593;
}
function poseMatrix(position,rotation){
  return new THREE.Matrix4().compose(new THREE.Vector3(...position).multiplyScalar(1000),new THREE.Quaternion(rotation[1],rotation[2],rotation[3],rotation[0]),new THREE.Vector3(1,1,1));
}
function jawTransform(degrees){
  return new THREE.Matrix4().makeTranslation(...jawPivot).multiply(new THREE.Matrix4().makeRotationAxis(jawAxis,THREE.MathUtils.degToRad(degrees)))
    .multiply(new THREE.Matrix4().makeTranslation(...jawPivot.map(v=>-v)));
}
function forceFromDrag(start,end){
  const delta=end.clone().sub(start),length=delta.length();
  return length<1e-6?new THREE.Vector3():delta.multiplyScalar(Math.min(20,length*.025)/length);
}
function draw(){if(visible&&!document.hidden&&renderer&&!disposed)renderer.render(scene,camera);}
function message(text){$('message').textContent=text;$('message').hidden=!text;}
function failure(error){errorMessage=error.message||String(error);message(errorMessage);document.body.dataset.error=errorMessage;}
async function json(url,options={}){
  const response=await fetch(url,{cache:'no-store',...options,signal:AbortSignal.timeout(20000)});
  const data=await response.json();if(!response.ok||data.ok===false)throw new Error(data.error?.message||data.error?.reason||data.message||data.error||`仿真请求失败 (${response.status})`);return data;
}
function command(op,extra={}){
  if(!session||disposed)return Promise.resolve();
  const run=async()=>{
    const data=await json('/api/simulation',{method:'POST',headers:{'Content-Type':'application/json','X-Token':token},body:JSON.stringify({session,op,...extra})});
    if(data.state)applyState(data.state);errorMessage='';delete document.body.dataset.error;message('');return data;
  };
  const result=requestChain.then(run);requestChain=result.catch(()=>{});return result;
}
function selectedSimulationControl(message,expectedModel,current){
  if(message?.version!==1||message.model_id!==expectedModel||
     !/^[a-f0-9]{64}$/i.test(message.expected_sha||'')||message.expected_sha!==current?.modelSha||
     !['run','pause','reset','stop'].includes(message.command))throw Error('所选仿真模型或 SHA 已变化，请重新选择模型');
  return message.command;
}
async function runLocalModel(){
  if(virtualMotion){clearHeld();await sendTwist([0,0,0]);}
  if(!state?.policyEnabled)await command('policy',{enabled:true});
  await command('pause',{paused:false});
}
async function pauseLocalModel(){
  if(virtualMotion){clearHeld();await sendTwist([0,0,0]);}
  await command('pause',{paused:true});
}
function virtualTwist(keys){
  keyboard.reset();
  for(const key of keys)keyboard.keydown({key},{allowHead:false});
  return keyboard.packet({fastAllowed:true,navAllowed:true}).twist;
}
function sendTwist(twist){
  desiredTwist=twist.slice();
  if(!motionSending)motionSending=(async()=>{
    await Promise.resolve();
    try{
      while(desiredTwist){const next=desiredTwist;desiredTwist=null;const key=JSON.stringify(next);
        if(key!==lastTwist){await command('command',{twist:next});lastTwist=key;}}
    }finally{motionSending=null;}
  })();
  return motionSending;
}
function paintKeys(){const keys=new Set(heldInputs.values());for(const b of document.querySelectorAll('[data-motion]')){const pressed=keys.has(b.dataset.motion);b.dataset.pressed=String(pressed);b.setAttribute('aria-pressed',String(pressed));}}
function clearHeld(){heldInputs.clear();lastTwist='';paintKeys();}
function manualMotion(){
  paintKeys();
  if(!virtualMotion||functionBusy||disposed||!visible||document.hidden||!state?.policyEnabled||state.paused)return;
  sendTwist(virtualTwist(new Set(heldInputs.values()))).catch(error=>{clearHeld();failure(error);});
}
async function resetLocalModel(){
  clearHeld();if(virtualMotion)await sendTwist([0,0,0]);
  await command('reset');frameView();
}
async function performAction(action){
  if(functionBusy||disposed||!session)return;
  functionBusy=true;
  try{await action();}catch(error){failure(error);}finally{functionBusy=false;}
}
function bindKeyboard(){
  const toggle=()=>performAction(()=>state?.policyEnabled&&!state.paused?pauseLocalModel():runLocalModel());
  const stopMotion=()=>{clearHeld();if(virtualMotion)sendTwist([0,0,0]).catch(failure);};
  $('play').onclick=toggle;
  $('reset').onclick=()=>performAction(resetLocalModel);
  $('drop').onclick=()=>performAction(async()=>{clearHeld();if(virtualMotion)await sendTwist([0,0,0]);await command('drop');});
  $('motion-stop').onclick=stopMotion;
  $('virtual-controls').hidden=$('motion-stop').hidden=!virtualMotion;
  const motionKey=e=>({arrowup:'w',arrowdown:'s',arrowleft:'a',arrowright:'d'})[e.key.toLowerCase()]||e.key.toLowerCase();
  const eligible=e=>!disposed&&visible&&!document.hidden&&document.hasFocus()&&!e.isComposing&&!e.target?.isContentEditable&&!['INPUT','TEXTAREA','SELECT'].includes(e.target?.tagName);
  document.addEventListener('keydown',e=>{
    if(!eligible(e))return;
    const key=motionKey(e),source='key:'+(e.code||e.key);
    if([' ','space','r','escape'].includes(key)){
      e.preventDefault();if(e.repeat)return;
      if(key==='r')performAction(resetLocalModel);else if(key==='escape')stopMotion();else toggle();
    }else if(virtualMotion&&['w','s','a','d','shift'].includes(key)){
      e.preventDefault();
      if(e.repeat||functionBusy||!state?.policyEnabled||state.paused)return;
      heldInputs.set(source,key);manualMotion();
    }
  });
  document.addEventListener('keyup',e=>{const source='key:'+(e.code||e.key);if(heldInputs.delete(source)){e.preventDefault();manualMotion();}});
  for(const b of document.querySelectorAll('[data-motion]')){
    b.onpointerdown=e=>{if(e.button!==0||b.disabled)return;e.preventDefault();b.focus();b.setPointerCapture(e.pointerId);heldInputs.set('pointer:'+e.pointerId,b.dataset.motion);manualMotion();};
    const release=e=>{if(heldInputs.delete('pointer:'+e.pointerId))manualMotion();};
    b.onpointerup=release;b.onpointercancel=release;b.onlostpointercapture=release;
  }
  window.addEventListener('blur',()=>{if(!disposed&&session)stopMotion();});
}
function applyState(snapshot){
  if(!snapshot||!Array.isArray(snapshot.body_poses))return;
  if(state&&Number.isFinite(state.seq)&&Number.isFinite(snapshot.seq)&&snapshot.seq<state.seq)return;
  state=snapshot;const poses=new Map(snapshot.body_poses.map(p=>[p.name,p]));
  for(const entry of objects){
    const pose=poses.get(entry.body);if(!pose)throw new Error(`仿真缺少部件：${entry.body}`);
    entry.object.matrix.copy(poseMatrix(pose.position,pose.quaternion)).multiply(entry.geometryMatrix);
    if(entry.jaw)entry.object.matrix.multiply(jawTransform(snapshot.mouthCommandDeg||0));
    entry.object.matrixWorldNeedsUpdate=true;
  }
  const magnitude=Math.hypot(...(snapshot.force?.worldN||[0,0,0]));
  const activeLabel={walk:'键盘行走',getup:'跌倒起身',pick:'空载 Pick',roulade:'翻滚'}[snapshot.activeModel]||'策略未启动';
  $('status').textContent=`仅仿真 · ${snapshot.paused?'已暂停':activeLabel}`;
  $('status').title=`倾角 ${Number(snapshot.tiltDeg||0).toFixed(1)}° · 施力 ${magnitude.toFixed(1)} N · ${snapshot.recovery?.phase||'—'} · 嘴部指令 ${snapshot.mouthCommandDeg||0}°`;
  document.body.dataset.mouthCommandDeg=String(snapshot.mouthCommandDeg||0);
  $('play-label').textContent=snapshot.policyEnabled&&!snapshot.paused?'暂停':'运行';
  document.body.dataset.paused=String(snapshot.paused);document.body.dataset.policyEnabled=String(snapshot.policyEnabled);
  document.body.dataset.simTime=String(snapshot.simTime);document.body.dataset.policySteps=String(snapshot.policySteps||0);
  document.body.dataset.modelSha=snapshot.modelSha||'';
  document.body.dataset.activeModel=snapshot.activeModel||'';
  document.body.dataset.activeModelSha=snapshot.activeModelSha||'';
  document.body.dataset.recoveryPhase=snapshot.recovery?.phase||'';
  if(externalControls&&window.parent!==window)parent.postMessage({type:'duck-simulation-preview-state',version:1,
    model_id:modelId,model_sha:snapshot.modelSha,paused:snapshot.paused,policy_enabled:snapshot.policyEnabled},location.origin);
  if(virtualMotion){
    const [forward,,yaw]=snapshot.command||[0,0,0],ready=snapshot.policyEnabled&&!snapshot.paused;
    $('command-status').textContent=!ready?'先运行模型':yaw>0?'左转':yaw<0?'右转':forward>0?`前进 ${forward.toFixed(1)} m/s`:forward<0?'后退 0.3 m/s':'零指令 · 停步';
    document.body.dataset.command=JSON.stringify(snapshot.command||[0,0,0]);
    for(const b of document.querySelectorAll('[data-motion]'))b.disabled=!ready;
    $('motion-stop').disabled=!ready;
    if(!ready)clearHeld();
    const root=poses.get('trunk_base');
    if(root&&camera&&controls){const point=new THREE.Vector3(root.position[0]*1000,root.position[1]*1000,0);
      if(followPosition){const delta=point.clone().sub(followPosition);camera.position.add(delta);controls.target.add(delta);}
      followPosition=point;}
  }
  draw();
}
function frameView(){
  scene.updateMatrixWorld(true);const bounds=new THREE.Box3();for(const entry of objects)bounds.expandByObject(entry.object);
  const center=bounds.getCenter(new THREE.Vector3()),size=bounds.getSize(new THREE.Vector3()),distance=Math.max(size.x,size.y,size.z,200)*1.8;
  controls.target.copy(center);camera.position.copy(center).add(new THREE.Vector3(1.1,-1.6,.9).normalize().multiplyScalar(distance));camera.updateProjectionMatrix();controls.update();draw();
}
function ray(event){
  const rect=renderer.domElement.getBoundingClientRect();pointer.set((event.clientX-rect.left)/rect.width*2-1,-(event.clientY-rect.top)/rect.height*2+1);
  raycaster.setFromCamera(pointer,camera);return raycaster.ray;
}
function pointOnPlane(event){return ray(event).intersectPlane(drag.plane,new THREE.Vector3());}
function pointerDown(event){
  if(!state||drag||(event.button!==0&&event.button!==2))return;
  ray(event);scene.updateMatrixWorld(true);const hit=raycaster.intersectObjects(pickables,true)[0];if(!hit)return;
  const trunk=state.body_poses.find(p=>p.name==='trunk_base');if(!trunk)return;
  const root=new THREE.Vector3(...trunk.position).multiplyScalar(1000),normal=camera.getWorldDirection(new THREE.Vector3());
  drag={id:event.pointerId,mode:event.button===2?'grab':'push',version:0,start:hit.point.clone(),end:hit.point.clone(),root,plane:new THREE.Plane().setFromNormalAndCoplanarPoint(normal,hit.point)};
  renderer.domElement.setPointerCapture(event.pointerId);document.body.dataset.dragging='true';event.preventDefault();
  if(drag.mode==='grab')sendGrab(true);else{arrow.position.copy(drag.start);arrow.visible=false;}
}
function sendGrab(force=false){
  if(!drag||drag.mode!=='grab'||grabPending)return;
  const delay=50-(performance.now()-lastGrabAt);
  if(!force&&delay>0){if(grabTimer===null)grabTimer=setTimeout(()=>{grabTimer=null;sendGrab(true);},delay);return;}
  const position=drag.root.clone().add(drag.end.clone().sub(drag.start)).multiplyScalar(.001);
  position.x=THREE.MathUtils.clamp(position.x,-2,2);position.y=THREE.MathUtils.clamp(position.y,-2,2);position.z=THREE.MathUtils.clamp(position.z,.08,1);
  const current=drag,version=current.version;
  grabPending=true;lastGrabAt=performance.now();command('grab',{position:position.toArray()}).catch(failure).finally(()=>{grabPending=false;if(drag===current&&current.version!==version)sendGrab(true);});
}
function pointerMove(event){
  if(!drag||event.pointerId!==drag.id)return;const point=pointOnPlane(event);if(!point)return;drag.end.copy(point);drag.version++;
  if(drag.mode==='grab')sendGrab();else{
    const delta=drag.end.clone().sub(drag.start),length=delta.length();arrow.visible=length>2;
    if(arrow.visible){arrow.setDirection(delta.normalize());arrow.setLength(length,Math.min(20,length*.3),Math.min(10,length*.15));}draw();
  }event.preventDefault();
}
function pointerUp(event,cancelled=false){
  if(!drag||event.pointerId!==drag.id)return;const current=drag;drag=null;arrow.visible=false;document.body.dataset.dragging='false';
  clearTimeout(grabTimer);grabTimer=null;
  if(renderer.domElement.hasPointerCapture(event.pointerId))renderer.domElement.releasePointerCapture(event.pointerId);
  if(current.mode==='grab')command('release').catch(failure);else if(!cancelled){
    const force=forceFromDrag(current.start,current.end);
    if(force.length()>0)command('push',{force:force.toArray(),duration:.2,body:'trunk_base',point:current.start.clone().multiplyScalar(.001).toArray()}).catch(failure);
  }draw();
}
async function suspend(){visible=false;clearHeld();drag=null;clearTimeout(grabTimer);grabTimer=null;arrow.visible=false;document.body.dataset.dragging='false';if(session)await pauseLocalModel().catch(failure);}
async function closeSession(){const current=session;session=null;if(current){const response=await fetch('/api/simulation',{method:'POST',keepalive:true,headers:{'Content-Type':'application/json','X-Token':token},body:JSON.stringify({session:current,op:'close'})});if(!response.ok)throw Error('虚拟仿真关闭未确认');}}
document.addEventListener('visibilitychange',()=>{if(document.hidden)suspend();else{visible=true;draw();}});
window.addEventListener('message',event=>{if(event.origin!==location.origin||event.source!==parent||event.data?.type!=='duck-model-preview-visibility')return;if(event.data.visible===false)suspend();else if(event.data.visible===true){visible=true;draw();}});
window.addEventListener('message',async event=>{
  if(event.origin!==location.origin||event.source!==parent)return;
  if(event.data?.type==='duck-simulation-pause')await suspend();
  if(event.data?.type==='duck-simulation-control'&&externalControls){
    try{
      await initDone;if(disposed||!session)throw Error('仿真会话已关闭，请重新选择模型');
      const operation=selectedSimulationControl(event.data,modelId,state);
      if(operation!=='pause'&&(!visible||document.hidden))throw Error('模型页已隐藏');
      if(operation==='run')await runLocalModel();
      else if(operation==='pause')await pauseLocalModel();
      else if(operation==='reset')await resetLocalModel();
      else{clearHeld();if(virtualMotion)await sendTwist([0,0,0]);}
      parent.postMessage({type:'duck-simulation-control-result',model_id:modelId,ok:true},location.origin);
    }catch(error){parent.postMessage({type:'duck-simulation-control-result',model_id:modelId,ok:false,error:error.message},location.origin);failure(error);}
  }
  if(event.data?.type==='duck-simulation-close'){
    disposed=true;events?.close();
    try{await initDone;await requestChain;await closeSession();parent.postMessage({type:'duck-simulation-closed',requestId:event.data.requestId,ok:true},location.origin);}
    catch(error){parent.postMessage({type:'duck-simulation-closed',requestId:event.data.requestId,ok:false,error:error.message},location.origin);}
  }
});
window.addEventListener('pagehide',()=>{events?.close();closeSession().catch(()=>{});disposed=true;resizeObserver?.disconnect();controls?.dispose();renderer?.dispose();});
async function init(){
  $('hint').textContent='本地仿真 · 左键施力 / 右键抓起 / 中键旋转 / 滚轮缩放';
  if(!/^[a-zA-Z0-9_-]{1,80}$/.test(modelId||''))throw new Error('未选择模型');
  const [geometry,local,mouthModel]=await Promise.all([json(`/api/models/${encodeURIComponent(modelId)}/geometry`),json('/api/local-session'),json('/model-assets/model-data-mouth.json')]);
  token=local.token;if(!token)throw new Error('本地仿真会话不可用');
  if(!Array.isArray(geometry.records)||!geometry.records.length)throw new Error('模型几何不可用');
  const mouth=mouthModel.mouth_visualization,head=geometry.records.find(r=>r.id===mouth.parent);
  if(!head)throw new Error('模型缺少嘴部所属头部');
  jawPivot=mouth.pivot_head_local_m.map(v=>v*1000);jawAxis=new THREE.Vector3(...mouth.axis_head_local).normalize();
  head.file='/model-assets/'+mouth.head_mesh;
  geometry.records.push({...head,id:'mouth_jaw',file:'/model-assets/'+mouth.jaw_mesh});
  scene=new THREE.Scene();scene.background=new THREE.Color('#0b1722');scene.add(arrow);scene.add(new THREE.HemisphereLight(0xc9f2ff,0x16212c,2.6));
  for(const [color,intensity,position] of [[0xdff8ff,3.1,[350,-450,450]],[0x46c8e0,2.0,[-300,250,250]],[0xeab177,.65,[250,400,150]]]){const light=new THREE.DirectionalLight(color,intensity);light.position.set(...position);scene.add(light);}
  camera=new THREE.PerspectiveCamera(38,1,.1,10000);camera.up.set(0,0,1);
  renderer=new THREE.WebGLRenderer({antialias:true});renderer.setPixelRatio(Math.min(devicePixelRatio,1.5));renderer.outputColorSpace=THREE.SRGBColorSpace;renderer.toneMapping=THREE.ACESFilmicToneMapping;renderer.toneMappingExposure=1.05;host.prepend(renderer.domElement);
  controls=new OrbitControls(camera,renderer.domElement);controls.mouseButtons={LEFT:null,MIDDLE:THREE.MOUSE.ROTATE,RIGHT:null};controls.enablePan=false;controls.minDistance=80;controls.maxDistance=4000;controls.addEventListener('change',draw);
  const stl=new STLLoader(),obj=new OBJLoader();
  await Promise.all(geometry.records.map(async record=>{
    const material=new THREE.MeshStandardMaterial({color:modelColor(record),roughness:.40,metalness:.32,side:THREE.DoubleSide});let content;
    if(record.file.endsWith('.obj')){content=await obj.loadAsync(record.file);content.traverse(child=>{if(child.isMesh){child.geometry.scale(...record.mesh_scale.map(v=>v*1000));child.material=material;pickables.push(child);}});}
    else{const mesh=await stl.loadAsync(record.file);mesh.scale(...record.mesh_scale.map(v=>v*1000));mesh.computeVertexNormals();content=new THREE.Mesh(mesh,material);pickables.push(content);}
    const holder=new THREE.Group();holder.matrixAutoUpdate=false;holder.add(content);scene.add(holder);
    objects.push({body:record.body,object:holder,geometryMatrix:poseMatrix(record.geom_position_m,record.geom_quaternion_wxyz),jaw:record.id==='mouth_jaw'});
  }));
  const grid=new THREE.GridHelper(2000,50,0x315269,0x1b3345);grid.rotation.x=Math.PI/2;scene.add(grid);
  const ground=new THREE.Mesh(new THREE.PlaneGeometry(2000,2000),new THREE.MeshStandardMaterial({color:0x0c1c29,roughness:1,transparent:true,opacity:.65}));ground.position.z=-.5;scene.add(ground);
  resizeObserver=new ResizeObserver(()=>{camera.aspect=host.clientWidth/Math.max(1,host.clientHeight);camera.updateProjectionMatrix();renderer.setSize(host.clientWidth,Math.max(1,host.clientHeight));draw();});resizeObserver.observe(host);
  const created=await json('/api/simulation',{method:'POST',headers:{'Content-Type':'application/json','X-Token':token},body:JSON.stringify({op:'create',model_id:modelId})});
  session=created.session;if(!session)throw new Error('仿真会话未创建');if(disposed){await closeSession();return;}
  if(!Array.isArray(created.state?.body_poses))throw new Error('仿真未返回有效物理姿态');applyState(created.state);frameView();
  events=new EventSource(`/api/simulation/events?session=${encodeURIComponent(session)}`);
  events.addEventListener('state',event=>{try{applyState(JSON.parse(event.data));if(!errorMessage)message('');}catch(error){failure(error);}});
  events.addEventListener('error',()=>{clearHeld();pauseLocalModel().catch(failure);if(!errorMessage)message('仿真连接中断，重连后保持暂停，需手动启动。');});
  for(const id of ['play','drop','reset'])$(id).disabled=false;
  bindKeyboard();
  renderer.domElement.addEventListener('contextmenu',event=>event.preventDefault());
  renderer.domElement.addEventListener('pointerdown',pointerDown);renderer.domElement.addEventListener('pointermove',pointerMove);
  renderer.domElement.addEventListener('pointerup',event=>pointerUp(event));renderer.domElement.addEventListener('pointercancel',event=>pointerUp(event,true));
  document.body.dataset.loaded='true';document.body.dataset.parts=String(objects.length);message('');draw();if(document.hidden||!visible)await suspend();
}
initDone=init().catch(failure);
