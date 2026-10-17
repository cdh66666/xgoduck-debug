/* Frozen official q=0 reference. This file has no actuator or calibration API. */
import * as THREE from 'three';
import { OrbitControls } from '../model-assets/vendor/examples/jsm/controls/OrbitControls.js';
import { STLLoader } from '../model-assets/vendor/examples/jsm/loaders/STLLoader.js';

const $=id=>document.getElementById(id);
const parameters=new URLSearchParams(location.search);
if(parameters.get('embed')==='1')document.body.classList.add('embed');
const dark=parameters.get('theme')!=='light';
document.body.classList.toggle('duck-dark',dark);
const palette=dark?{
  background:'#08111b',tagBackground:'#101d2bf0',fixed:'#b8c6d3',align:'#eca45b',axis:'#54c9e8',axisLabel:'#dce8f5',front:'#899fb3',pointStroke:'#08111b',
  primary:0xeea65f,held:0xb8c6d3,child:0xcba87c,other:0x91a4b6,whole:0xa7b9c9,
}:{
  background:'#ffffff',tagBackground:'#fffffff0',fixed:'#4e6674',align:'#ae5919',axis:'#1680a3',axisLabel:'#116886',front:'#607785',pointStroke:'#ffffff',
  primary:0xee9a35,held:0x70828d,child:0xf2c48c,other:0xcbd5da,whole:0x667d8c,
};
const ids=[10,11,12,13,14,20,21,22,23,24,30,31,32,33,34];
const groups={left:ids.slice(0,5),right:ids.slice(5,10),head:ids.slice(10)};
const groupLabels={left:'左腿',right:'右腿',head:'头部'};
let pendingGroup=Object.hasOwn(groups,parameters.get('group'))?parameters.get('group'):null;
const tips={
  10:'顶视：11 在 10 前方，两轴心沿机身前后线对齐。',20:'顶视：21 在 20 前方，两轴心沿机身前后线对齐。',
  11:'前视：11、12 两轴心同高。',21:'前视：21、22 两轴心同高。',
  12:'托稳灰机身，绕12转橙大腿；侧看膝13在髋12后下方，对齐蓝虚线；嘴朝向为前。',22:'托稳灰机身，绕22转橙大腿；侧看膝23在髋22后下方，对齐蓝虚线；嘴朝向为前。',
  13:'侧视：膝、踝两轴心上下对齐。不是把大小腿拉成直线。',23:'侧视：膝、踝两轴心上下对齐。不是把大小腿拉成直线。',
  14:'侧视：脚底前后水平；蓝线只作水平参考，不是接触点，不必双脚贴地。',24:'侧视：脚底前后水平；蓝线只作水平参考，不是接触点，不必双脚贴地。',
  30:'侧视：颈根、颈上端两轴心上下对齐，间距 50 mm。',
  31:'侧视：31、32 两轴心上下对齐，头托架按橙色轮廓摆。',
  32:'顶视：头壳中线沿机身前后方向，不左右扭头。',
  33:'前视：灰座托住，橙壳左右水平。头壳透视显示，方便看清里面的固定件。',
  34:'嘴34：保持照片的轻闭合状态，读取并记录此时的编码值。',
};
const successors={10:11,11:12,12:13,13:14,20:21,21:22,22:23,23:24,30:31,31:32,32:33};
const scene=new THREE.Scene();scene.background=new THREE.Color(palette.background);
scene.add(new THREE.HemisphereLight(dark?0xd8e8f7:0xffffff,dark?0x40556b:0x8494a0,dark?2.2:2.6));
const light=new THREE.DirectionalLight(dark?0xe8f2ff:0xffffff,dark?2.5:2.8);light.position.set(.4,-.5,.7);scene.add(light);
const back=new THREE.DirectionalLight(dark?0x81bfd8:0xffffff,dark?1.5:1.1);back.position.set(-.3,.5,.2);scene.add(back);
const meshes=new Map(),centers=new Map(),frames=[];
let data=null,selected=null,selectedId=Number(parameters.get('id'))||10,pendingId=selectedId,box=null,held=null;
let annotationPoints=[];
let whole=false,groupView=null;
const svgNS='http://www.w3.org/2000/svg';
function svgElement(tag,attributes={},content=''){
  const element=document.createElementNS(svgNS,tag);
  for(const [key,value] of Object.entries(attributes))element.setAttribute(key,String(value));
  if(content)element.textContent=content;return element;
}
function descendants(name){const set=new Set([name]);for(const record of data.records)if(record.parent===name)for(const child of descendants(record.name))set.add(child);return set;}
function mainDirection(id){
  if([10,20,32].includes(id))return {direction:new THREE.Vector3(0,0,1),up:new THREE.Vector3(1,0,0),label:'顶视'};
  if([11,21,33].includes(id))return {direction:new THREE.Vector3(1,0,0),up:new THREE.Vector3(0,0,1),label:'前视'};
  return {direction:new THREE.Vector3(0,id<20||id>=30?1:-1,0),up:new THREE.Vector3(0,0,1),label:id<20||id>=30?'左侧视':'右侧视'};
}
function fit(frame){
  if(!box)return;
  const rect=frame.viewport.getBoundingClientRect();if(rect.width<2||rect.height<2)return;
  const center=box.getCenter(new THREE.Vector3());
  const size=box.getSize(new THREE.Vector3());
  const view=(whole||groupView)?{direction:new THREE.Vector3(.8,.95,.5).normalize(),up:new THREE.Vector3(0,0,1),label:'拖动旋转'}:mainDirection(selectedId);
  const aspect=rect.width/rect.height;
  // Fit all eight box corners after selecting the view orientation.
  frame.camera.up.copy(view.up);frame.camera.position.copy(center).addScaledVector(view.direction,.55);frame.camera.lookAt(center);frame.camera.updateMatrixWorld();
  const cameraBounds=new THREE.Box3();
  for(const x of [box.min.x,box.max.x])for(const y of [box.min.y,box.max.y])for(const z of [box.min.z,box.max.z])cameraBounds.expandByPoint(new THREE.Vector3(x,y,z).applyMatrix4(frame.camera.matrixWorldInverse));
  const projected=cameraBounds.getSize(new THREE.Vector3());
  const height=Math.max(projected.y,projected.x/aspect,.026)*1.42;
  frame.camera.left=-height*aspect/2;frame.camera.right=height*aspect/2;frame.camera.top=height/2;frame.camera.bottom=-height/2;
  frame.camera.zoom=1;frame.camera.updateProjectionMatrix();frame.controls.target.copy(center);frame.controls.update();
  frame.renderer.setSize(rect.width,rect.height);$(frame.title).textContent=view.label;
  frame.baseDirection=view.direction.clone();
  annotate(frame);
}
function project(frame,point){const rect=frame.viewport.getBoundingClientRect();const v=point.clone().project(frame.camera);return {x:(v.x+1)*rect.width/2,y:(1-v.y)*rect.height/2};}
function arrow(svg,x1,y1,x2,y2,color){
  svg.append(svgElement('path',{d:`M${x1} ${y1}L${x2} ${y2}`,stroke:color,'stroke-width':1.8,fill:'none'}));
  const angle=Math.atan2(y2-y1,x2-x1),length=8;
  svg.append(svgElement('path',{d:`M${x2-length*Math.cos(angle-.45)} ${y2-length*Math.sin(angle-.45)}L${x2} ${y2}L${x2-length*Math.cos(angle+.45)} ${y2-length*Math.sin(angle+.45)}`,stroke:color,'stroke-width':1.8,fill:'none'}));
}
function tag(svg,label,x,y,color){svg.append(svgElement('rect',{x,y,width:116,height:25,rx:4,fill:palette.tagBackground,stroke:color,'stroke-width':1}));svg.append(svgElement('text',{x:x+8,y:y+17,fill:color,style:`fill:${color}`,'font-size':12},label));}
function fixedAnchor(frame){
  if(![11,21,32,33].includes(selectedId))return project(frame,centers.get(held.name));
  const parentMesh=meshes.get(held.name),primary=meshes.get(selected.name);
  // Pick a real, visible parent surface instead of its centre hidden by the child.
  const positions=parentMesh.geometry.attributes.position,ray=new THREE.Raycaster();
  const rect=frame.viewport.getBoundingClientRect(),candidates=[];
  for(let i=0,step=Math.max(1,Math.floor(positions.count/160));i<positions.count;i+=step){
    const world=new THREE.Vector3().fromBufferAttribute(positions,i).applyMatrix4(parentMesh.matrixWorld);
    const p=project(frame,world);if(p.x<12||p.x>rect.width-12||p.y<45||p.y>rect.height-12)continue;
    candidates.push({world,p,cost:Math.hypot(p.x-90,p.y-34)});
  }
  candidates.sort((a,b)=>a.cost-b.cost);
  for(const {world,p} of candidates){
    let covered=false;
    if(selectedId!==33)for(const [dx,dy] of [[0,0],[-8,0],[8,0],[0,-8],[0,8]]){
      ray.setFromCamera(new THREE.Vector2((p.x+dx)/rect.width*2-1,1-(p.y+dy)/rect.height*2),frame.camera);
      if(ray.intersectObject(primary,false).length){covered=true;break;}
    }
    if(covered)continue;
    return p;
  }
  return null;
}
function shellAnchor(frame){
  const mesh=meshes.get(selected.name),positions=mesh.geometry.attributes.position;
  const rect=frame.viewport.getBoundingClientRect();let best=null,cost=Infinity;
  for(let i=0,step=Math.max(1,Math.floor(positions.count/160));i<positions.count;i+=step){
    const p=project(frame,new THREE.Vector3().fromBufferAttribute(positions,i).applyMatrix4(mesh.matrixWorld));
    if(p.x<12||p.x>rect.width-12||p.y<45||p.y>rect.height-12)continue;
    const candidate=Math.hypot(p.x-(rect.width-90),p.y-34);if(candidate<cost){cost=candidate;best=p;}
  }
  return best;
}
function annotate(frame){
  const rect=frame.viewport.getBoundingClientRect(),w=rect.width,h=rect.height;frame.svg.setAttribute('viewBox',`0 0 ${w} ${h}`);frame.svg.replaceChildren();
  if(!selected||!box)return;
  if(!whole&&!groupView){
    const child=selectedId===33?(shellAnchor(frame)||project(frame,centers.get(selected.name))):project(frame,centers.get(selected.name));
    const parent=held?fixedAnchor(frame):null;
    if(held){tag(frame.svg,selectedId===33?'灰座：托住不动':'灰：托住不动',8,8,palette.fixed);if(parent)arrow(frame.svg,90,34,parent.x,parent.y,palette.fixed);}
    tag(frame.svg,selectedId===33?'橙壳：摆到这里':'橙：摆到这里',Math.max(126,w-124),8,palette.align);
    if(selectedId!==34)arrow(frame.svg,Math.max(160,w-90),34,child.x,child.y,palette.align);
  }
  const projections=annotationPoints.map(point=>({...project(frame,point.position),label:point.label}));
  if(projections.length===2){frame.svg.append(svgElement('path',{d:`M${projections[0].x} ${projections[0].y}L${projections[1].x} ${projections[1].y}`,stroke:palette.axis,'stroke-width':2,'stroke-dasharray':'5 4',fill:'none'}));}
  for(const p of projections){frame.svg.append(svgElement('circle',{cx:p.x,cy:p.y,r:4,fill:palette.axis,stroke:palette.pointStroke,'stroke-width':1}));frame.svg.append(svgElement('text',{x:p.x+7,y:p.y-7,fill:palette.axisLabel,'font-size':12},p.label));}
  // A projected body +X direction identifies front even if the user rotates the camera.
  const c=box.getCenter(new THREE.Vector3()),span=box.getSize(new THREE.Vector3()).length()*.19;
  const start=project(frame,c),end=project(frame,c.clone().add(new THREE.Vector3(span,0,0)));
  const dx=end.x-start.x,dy=end.y-start.y,length=Math.hypot(dx,dy);
  if(length>2){const ux=dx/length,uy=dy/length,x=w-68,y=h-28;arrow(frame.svg,x-ux*20,y-uy*20,x+ux*20,y+uy*20,palette.front);frame.svg.append(svgElement('text',{x:w-104,y:h-8,fill:palette.front,'font-size':12},'嘴朝向为前'));}
}
function createFrame(kind){
  const viewport=$(`zero-${kind}-viewport`),svg=$(`zero-${kind}-annotation`);
  const renderer=new THREE.WebGLRenderer({antialias:true});renderer.setPixelRatio(Math.min(devicePixelRatio,1.5));viewport.prepend(renderer.domElement);
  if(dark){renderer.toneMapping=THREE.ACESFilmicToneMapping;renderer.toneMappingExposure=1;}
  const camera=new THREE.OrthographicCamera(-.1,.1,.1,-.1,.001,20);camera.up.set(0,0,1);
  const controls=new OrbitControls(camera,renderer.domElement);controls.enableDamping=!matchMedia('(prefers-reduced-motion: reduce)').matches;controls.minZoom=.4;controls.maxZoom=8;
  const frame={kind,viewport,svg,renderer,camera,controls,title:kind==='main'?'zero-view-main-title':'zero-view-iso-title'};
  frame.dirty=true;
  controls.addEventListener('start',()=>$(frame.title).textContent='旋转查看');
  controls.addEventListener('change',()=>{frame.dirty=true;});
  new ResizeObserver(()=>fit(frame)).observe(viewport);frames.push(frame);
}
createFrame('main');

function select(id,notify=false){
  if(!data){pendingId=id;return;}
  const joint=data.joints.find(row=>row.id===id);if(!joint)return;
  if(selected&&selectedId===id&&!whole&&!groupView)return;
  whole=false;groupView=null;document.body.dataset.group='';
  document.querySelector('.zero-legend .fixed').textContent='灰色：托住不动';document.querySelector('.zero-legend .align').textContent='橙色：摆到图中位置';
  $('zero-whole').setAttribute('aria-pressed','false');$('zero-whole').textContent='整机零位';
  selectedId=id;selected=joint;$('zero-joint').value=String(id);
  document.body.dataset.renderedId='pending';
  for(const frame of frames)frame.dirty=true;
  const record=data.records.find(row=>row.name===joint.name);held=data.records.find(row=>row.name===record?.parent)||null;
  const children=descendants(joint.name);
  for(const [name,mesh] of meshes){
    const primary=name===joint.name,fixed=name===held?.name;
    mesh.visible=true;mesh.material.color.setHex(primary?palette.primary:fixed?palette.held:children.has(name)?palette.child:palette.other);
    mesh.material.opacity=primary?(id===33?.30:1):fixed?(id===33?.95:dark?.60:.45):children.has(name)?.22:dark?.08:.065;
    mesh.material.depthWrite=primary&&id!==33;mesh.renderOrder=primary?3:fixed?2:1;
  }
  box=new THREE.Box3();
  box.expandByObject(meshes.get(joint.name));if(held)box.expandByObject(meshes.get(held.name));
  annotationPoints=[];
  const origin=data.poses['q-zero'].joints[String(id)]?.origin_m;
  const nextId=successors[id],next=data.poses['q-zero'].joints[String(nextId)]?.origin_m;
  if(origin)annotationPoints.push({position:new THREE.Vector3(...origin),label:`轴心 ${id}`});
  if(next){annotationPoints.push({position:new THREE.Vector3(...next),label:`轴心 ${nextId}`});box.expandByPoint(new THREE.Vector3(...next));}
  if([14,24].includes(id)){
    const footBox=new THREE.Box3().setFromObject(meshes.get(joint.name)),center=footBox.getCenter(new THREE.Vector3());
    annotationPoints=[{position:new THREE.Vector3(footBox.min.x,center.y,footBox.min.z),label:'后'},{position:new THREE.Vector3(footBox.max.x,center.y,footBox.min.z),label:'前'}];
  }
  if(id===34){held=data.records.find(row=>row.name==='head_yaw');box=new THREE.Box3().setFromObject(meshes.get('head_roll'));annotationPoints=[];}
  document.querySelector('.zero-views').hidden=id===34;$('zero-mouth').hidden=id!==34;
  document.querySelector('.zero-legend').hidden=id===34;
  $('zero-title').textContent=`ID ${id} · ${id===34?'嘴巴（轻闭合）':joint.label}`;$('zero-tip').textContent=tips[id];
  document.body.dataset.loaded='true';document.body.dataset.selectedId=String(id);document.body.dataset.referencePose=id===34?'user-confirmed-closed-mouth':'q-zero';document.body.dataset.meshCount=String(meshes.size);
  $('zero-scope').textContent=id===34?'按已确认照片轻闭嘴；编码仅记录，掉电/方向/模型应用待核验。':'官方 0° 姿势参考；编码仅记录，掉电/方向/模型应用待核验。';
  if(id===34){const photo=$('zero-mouth').querySelector('img');if(photo.complete&&photo.naturalWidth>0)document.body.dataset.renderedId='34';else photo.addEventListener('load',()=>{if(selectedId===34)document.body.dataset.renderedId='34';},{once:true});}
  for(const frame of frames)fit(frame);
  if(notify&&window.parent!==window)window.parent.postMessage({type:'zero-guide-joint-selected',id},location.origin);
}
function showWhole(){
  if(!data||meshes.size!==15)return;
  if(whole){select(selectedId);return;}
  whole=true;groupView=null;document.body.dataset.group='';document.body.dataset.renderedId='pending';
  document.querySelector('.zero-views').hidden=false;$('zero-mouth').hidden=true;
  document.querySelector('.zero-legend').hidden=true;annotationPoints=[];box=new THREE.Box3();
  for(const mesh of meshes.values()){mesh.material.color.setHex(palette.whole);mesh.material.opacity=1;mesh.material.depthWrite=true;mesh.renderOrder=1;box.expandByObject(mesh);}
  $('zero-title').textContent='整机零位参考';$('zero-whole').textContent='返回这颗';$('zero-whole').setAttribute('aria-pressed','true');
  $('zero-tip').textContent='托住机身按图摆姿；嘴巴轻闭合。双脚不必贴桌。';
  $('zero-scope').textContent='整机和逐颗图使用同一模型零位。摆姿只作参考，记录后仍需复测。';
  document.body.dataset.referencePose='q-zero-whole';for(const frame of frames){frame.dirty=true;fit(frame);}
}
function showGroup(group){
  if(!Object.hasOwn(groups,group))return;
  if(!data||meshes.size!==15){pendingGroup=group;return;}
  if(!groups[group].includes(selectedId))select(groups[group][0]);
  whole=false;groupView=group;pendingGroup=null;annotationPoints=[];box=new THREE.Box3();
  const names=new Set();
  for(const id of groups[group]){const joint=data.joints.find(row=>row.id===id);if(joint)for(const name of descendants(joint.name))names.add(name);
    const origin=data.poses['q-zero'].joints[String(id)]?.origin_m;
    if(origin&&id!==34)annotationPoints.push({position:new THREE.Vector3(...origin),label:String(id)});
  }
  for(const [name,mesh] of meshes){const highlighted=names.has(name);mesh.visible=true;mesh.material.color.setHex(highlighted?palette.primary:palette.whole);
    mesh.material.opacity=highlighted?1:dark?.38:.30;mesh.material.depthWrite=highlighted;mesh.renderOrder=highlighted?2:1;box.expandByObject(mesh);
  }
  document.querySelector('.zero-views').hidden=false;$('zero-mouth').hidden=true;document.querySelector('.zero-legend').hidden=false;
  document.querySelector('.zero-legend .fixed').textContent='灰色：其他部位';document.querySelector('.zero-legend .align').textContent='橙色：'+groupLabels[group]+' 5 颗';
  $('zero-title').textContent=groupLabels[group]+'零位';$('zero-whole').textContent='整机零位';$('zero-whole').setAttribute('aria-pressed','false');
  $('zero-tip').textContent=group==='head'?'托住机身，按橙色摆好头部；嘴34按实物照轻闭合。':'托住机身，按橙色摆好'+groupLabels[group]+'；双脚不必贴桌。';
  $('zero-scope').textContent='只记录当前组原生编码；掉电/方向/模型应用待核验。点编号看对齐方式。';
  document.body.dataset.group=group;document.body.dataset.referencePose='q-zero-group';document.body.dataset.renderedId='pending';
  for(const frame of frames){frame.dirty=true;fit(frame);}
}
$('zero-whole').addEventListener('click',showWhole);
$('zero-joint').addEventListener('change',()=>select(Number($('zero-joint').value),true));
$('zero-previous').onclick=()=>select(ids[Math.max(0,ids.indexOf(selectedId)-1)],true);
$('zero-next').onclick=()=>select(ids[Math.min(ids.length-1,ids.indexOf(selectedId)+1)],true);
$('zero-reset').onclick=()=>frames.forEach(fit);
window.addEventListener('message',event=>{
  if(event.origin!==location.origin||event.source!==window.parent)return;
  if(event.data?.type==='zero-guide-select'&&ids.includes(event.data.id))select(event.data.id);
  if(event.data?.type==='zero-guide-group'&&Object.hasOwn(groups,event.data.group))showGroup(event.data.group);
});
function animate(){requestAnimationFrame(animate);for(const frame of frames){if(document.hidden||frame.viewport.clientWidth<2||frame.viewport.clientHeight<2)continue;frame.controls.update();if(frame.dirty){frame.renderer.render(scene,frame.camera);if(selected)annotate(frame);frame.renderedId=selectedId;frame.dirty=false;}}if((whole||groupView||selectedId!==34)&&frames.every(frame=>frame.renderedId===selectedId))document.body.dataset.renderedId=String(selectedId);}
requestAnimationFrame(animate);
try{
  const response=await fetch('../model-assets/model-data.json');if(!response.ok)throw Error('模型清单未读取');data=await response.json();
  if(!data.poses?.['q-zero']||data.records.length!==15)throw Error('零角基线不完整');
  const loader=new STLLoader();
  await Promise.all(data.records.map(async record=>{
    const geometry=await loader.loadAsync(`../model-assets/${record.name}.stl`);geometry.computeVertexNormals();
    const material=new THREE.MeshStandardMaterial({color:palette.other,roughness:dark?.7:.8,metalness:dark?.04:.03,transparent:true});
    const mesh=new THREE.Mesh(geometry,material);mesh.name=record.name;mesh.matrixAutoUpdate=false;
    mesh.matrix.set(...data.poses['q-zero'].records[record.name].flat());mesh.matrixWorldNeedsUpdate=true;scene.add(mesh);meshes.set(record.name,mesh);
  }));
  scene.updateMatrixWorld(true);
  for(const [name,mesh] of meshes)centers.set(name,new THREE.Box3().setFromObject(mesh).getCenter(new THREE.Vector3()));
  for(const joint of data.joints)$('zero-joint').add(new Option(`${joint.id} · ${joint.id===34?'嘴巴（轻闭合）':joint.label}`,String(joint.id)));
  select(pendingId);
  if(pendingGroup)showGroup(pendingGroup);else if(parameters.get('whole')==='1')showWhole();
  if(window.parent!==window)window.parent.postMessage({type:'zero-guide-ready',version:data.version,meshes:meshes.size,pose:'q-zero',view:groupView?'group':whole?'whole':'joint',group:groupView,id:selectedId},location.origin);
}catch(error){$('zero-tip').textContent='图未加载：'+error.message;document.body.dataset.loaded='false';console.error(error);}
