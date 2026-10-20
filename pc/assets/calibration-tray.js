import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
import {STLLoader} from 'three/addons/loaders/STLLoader.js';

const scene=new THREE.Scene();scene.background=new THREE.Color('#0b1823');
const camera=new THREE.PerspectiveCamera(35,innerWidth/innerHeight,.1,5000);camera.up.set(0,0,1);
const renderer=new THREE.WebGLRenderer({antialias:true,preserveDrawingBuffer:true});renderer.setPixelRatio(Math.min(devicePixelRatio,2));renderer.setSize(innerWidth,innerHeight);document.body.prepend(renderer.domElement);
const controls=new OrbitControls(camera,renderer.domElement);controls.enableDamping=true;
scene.add(new THREE.HemisphereLight(0xeef8ff,0x486575,2.2));
const light=new THREE.DirectionalLight(0xffffff,3);light.position.set(140,-120,300);scene.add(light);
const tray=new THREE.Group(),duck=new THREE.Group();scene.add(tray,duck);
const loader=new STLLoader();let loaded=false,total=0;
const status=document.querySelector('#status');
if(parent!==window)document.querySelector('header h1').hidden=true;
function fit(){
  if(!loaded)return;
  const bounds=new THREE.Box3().setFromObject(tray);if(duck.visible)bounds.expandByObject(duck);
  const center=bounds.getCenter(new THREE.Vector3()),size=bounds.getSize(new THREE.Vector3()).length();
  controls.target.copy(center);camera.position.copy(center).addScaledVector(new THREE.Vector3(1,-.85,1.1).normalize(),size*1.75);
  camera.near=.1;camera.far=Math.max(5000,size*20);camera.updateProjectionMatrix();controls.update();
}
function visible(value){duck.visible=value;document.querySelector('#toggle-duck').textContent=value?'只看托盘':'显示小鸭子';document.querySelector('#restore').hidden=value;status.textContent=`R7 · 托盘 1 件${value?` + 小鸭子 ${total-1} 件`:` · 已隐藏 ${total-1} 件`}`;document.body.dataset.duckVisible=String(value);}
document.querySelector('#toggle-duck').onclick=()=>visible(!duck.visible);
document.querySelector('#restore').onclick=()=>visible(true);
document.querySelector('#reset').onclick=fit;
try{
  const response=await fetch('/assets/calibration-tray-r7/manifest.json',{cache:'no-store'});if(!response.ok)throw Error('无法读取托盘版本');
  const manifest=await response.json();if(manifest.version!=='R7-20261020'||manifest.geometry_modified!==false)throw Error('托盘版本不匹配');
  total=manifest.records.length;
  await Promise.all(manifest.records.map(async record=>{
    const geometry=await loader.loadAsync('/assets/calibration-tray-r7/'+record.file);geometry.computeVertexNormals();
    const mesh=new THREE.Mesh(geometry,new THREE.MeshStandardMaterial({color:record.color,roughness:.65,metalness:.08,side:THREE.DoubleSide}));mesh.name=record.id;(record.kind==='tray'?tray:duck).add(mesh);
  }));
  const bounds=new THREE.Box3().setFromObject(tray);
  const grid=new THREE.GridHelper(500,20,0x35586b,0x1d3544);grid.rotation.x=Math.PI/2;grid.position.z=bounds.min.z-.2;scene.add(grid);
  loaded=true;visible(true);fit();document.body.dataset.loaded='true';document.body.dataset.meshCount=String(total);
}catch(error){status.textContent='模型未加载：'+error.message;document.body.dataset.error=error.message;}
addEventListener('resize',()=>{camera.aspect=innerWidth/innerHeight;camera.updateProjectionMatrix();renderer.setSize(innerWidth,innerHeight);});
addEventListener('keydown',event=>{if(event.key==='Escape'&&parent!==window){event.preventDefault();parent.postMessage({type:'duck-tray-escape'},location.origin);}});
renderer.setAnimationLoop(()=>{controls.update();renderer.render(scene,camera);});
