/* Stateful keyboard command math only. No network, DOM, timers, or device IO. */
((root)=>{
 'use strict';
 const HEAD_KEYS=new Set(['i','k','j','l','u','o']);
 const MOTION_KEYS=new Set(['w','s','a','d','shift']);
 const ALL_KEYS=new Set([...HEAD_KEYS,...MOTION_KEYS,'space']);
 const opposite={i:'k',k:'i',j:'l',l:'j',u:'o',o:'u'};
 const clamp=(value,lo,hi)=>Math.min(hi,Math.max(lo,Math.round(value*1e9)/1e9));
 function keyName(event){return event.code==='Space'?'space':String(event.key||'').toLowerCase();}
 function create({headStep=.05,depthStep=.05,mouthStep=3,repeatMs=120,now=()=>performance.now()}={}){
  const held=new Set(),steppedAt=new Map();let head=[0,0,0,0],mouth=0,manualPickDepth=0,pitchLevel=0;
  function reset(){held.clear();steppedAt.clear();head=[0,0,0,0];mouth=0;manualPickDepth=0;pitchLevel=0;}
  function pitch(value){pitchLevel=clamp(value,-1,1);manualPickDepth=Math.max(0,pitchLevel);head[0]=Math.max(0,-pitchLevel);head[1]=Math.min(0,pitchLevel);}
  function clearMotion(){for(const key of MOTION_KEYS)held.delete(key);}
  function releaseAll(){held.clear();steppedAt.clear();}
  function keydown(event,{allowHead=true}={}){
   const key=keyName(event);if(!ALL_KEYS.has(key))return {handled:false,changed:false,roll:false};
   const first=!held.has(key);held.add(key);
   if(key==='space')return {handled:true,changed:false,roll:first&&!event.repeat};
   if(MOTION_KEYS.has(key)){
    const resetDepth=['w','s','a','d'].includes(key)&&manualPickDepth!==0;
    if(resetDepth)pitch(0);
    return {handled:true,changed:first||resetDepth,roll:false};
   }
   if(!allowHead||held.has(opposite[key]))return {handled:true,changed:false,roll:false};
   const stamp=now();
   if((event.repeat||!first)&&stamp-(steppedAt.get(key)??-Infinity)<repeatMs)return {handled:true,changed:false,roll:false};
   steppedAt.set(key,stamp);
   const before=head.join(',')+'/'+mouth+'/'+manualPickDepth;
   if(key==='i'||key==='k'){
    const sign=key==='k'?1:-1;
    const requested=pitchLevel+depthStep*sign;
    pitch(['w','s','a','d'].some(value=>held.has(value))?Math.min(0,requested):requested);
   }else if(key==='j'||key==='l')head[2]=clamp(head[2]+(key==='j'?headStep:-headStep),-1,1);
   else mouth=clamp(mouth+(key==='u'?mouthStep:-mouthStep),0,30);
   return {handled:true,changed:before!==head.join(',')+'/'+mouth+'/'+manualPickDepth,roll:false};
  }
  function keyup(event){return held.delete(keyName(event));}
  function centerHead(){head=[0,0,0,0];manualPickDepth=0;pitchLevel=0;for(const key of ['i','k','j','l']){held.delete(key);steppedAt.delete(key);}}
  function snapshot(){return {head:head.slice(),mouth,manualPickDepth,pitchLevel,held:[...held]};}
  function packet({fastAllowed=false,navAllowed=false,pickActive=false,pickMouth=0}={}){
   const forwards=held.has('w')&&!held.has('s'),backwards=held.has('s')&&!held.has('w');
   const yaw=Number(held.has('a'))-Number(held.has('d'));
   const navigation=(yaw!==0||backwards)&&navAllowed;
   const specialist=yaw===0&&forwards&&fastAllowed;
   const fast=specialist&&held.has('shift');
   const twist=yaw!==0?[0,0,navAllowed?yaw:0]:[specialist?(fast?.3:.2):navigation&&backwards?-.3:0,0,0];
   return {twist,navigation_specialist:navigation,forward_specialist:specialist,fast_forward:fast,manual_pick_depth:manualPickDepth,
           head:head.slice(),mouth:pickActive?clamp(Number.isFinite(pickMouth)?pickMouth:0,0,30):mouth};
  }
  return {keydown,keyup,reset,clearMotion,releaseAll,centerHead,snapshot,packet};
 }
 root.DuckKeyboard={create,HEAD_KEYS,keyName};
})(typeof window==='object'?window:globalThis);
