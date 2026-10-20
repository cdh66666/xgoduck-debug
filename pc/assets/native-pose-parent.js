/* Read-only measured joint display. Never calls a device action endpoint. */
(function(scope){
 'use strict';
 const IDS=[10,11,12,13,14,20,21,22,23,24,30,31,32,33,34];
 const hash=value=>typeof value==='string'&&/^[a-f0-9]{64}$/.test(value);
 const validRecord=r=>r&&Number.isInteger(r.position)&&r.position>=-32767&&r.position<=32767&&
  r.signed_steps===r.position&&[0,4].includes(r.mode)&&
  (r.raw_word===(r.position<0?32768-r.position:r.position)||r.position===0&&r.raw_word===0x8000)&&
  (r.mode===4?r.coordinate_domain==='native-sign-magnitude-v1':r.coordinate_domain==='single-turn-0-4095'&&r.position>=0&&r.position<=4095);
 function zeroMetadata(s){
  if(!s||s.complete!==true||s.native_published!==true||s.published!==true||s.native_file?.verified!==true||
   s.native_file.path!=='data/zero_pos.native.json'||!hash(s.native_file.sha256)||s.native_file.sha256==='0'.repeat(64)||s.group_file?.verified!==true||!hash(s.group_file.sha256)||
   s.native_file.group_records_sha256!==s.group_file.sha256||!IDS.every(id=>validRecord(s.records?.[id])))return null;
  return {sha:s.native_file.sha256,zeros:IDS.map(id=>s.records[id].position),modes:IDS.map(id=>s.records[id].mode)};
 }
 function decodeObservation(encoded,feedbackAge,decode64){
  if(typeof encoded!=='string'||!/^[A-Za-z0-9+/]{128}$/.test(encoded)||!Number.isFinite(feedbackAge)||feedbackAge<0||feedbackAge>=150)return null;
  let raw;try{raw=decode64(encoded);}catch{return null;}
  if(raw.length!==96||raw.slice(0,3)!=='JP1')return null;
  const bytes=Uint8Array.from(raw,c=>c.charCodeAt(0)),view=new DataView(bytes.buffer);
  const ref=view.getUint32(11,true)/1000,extra=feedbackAge-ref;
  if(ref>=150||extra<-.001)return null;
  const modes=view.getUint16(15,true),valid=view.getUint16(17,true);
  if(modes&0x8000||valid&0x8000||(modes&~valid)!==0||view.getUint32(3,true)===0)return null;
  const positions=IDS.map((_,i)=>view.getInt16(19+2*i,true));
  if(positions.some(n=>n===-32768))return null;
  for(let i=0;i<15;i++){
   const age=view.getUint8(49+i);
   if(valid&(1<<i)){if(age>=150||!(modes&(1<<i))&&(positions[i]<0||positions[i]>4095))return null;}
   else if(positions[i]!==0||age!==255)return null;
  }
  const zeroSha=Array.from(bytes.slice(64),v=>v.toString(16).padStart(2,'0')).join('');
  if(zeroSha==='0'.repeat(64))return null;
  return {boot_id:view.getUint32(3,true),sequence:view.getUint32(7,true),age_ms:Math.max(0,extra),
   positions,modes:IDS.map((_,i)=>modes&(1<<i)?4:0),
   position_age_ms:IDS.map((_,i)=>valid&(1<<i)?view.getUint8(49+i)+Math.max(0,extra):255),
   zero_sha256:zeroSha};
 }
 function posePacket(detail,metadata,now,corrections,decode64){
  const s=detail?.status,checked=detail?.checkedAt;
  if(detail?.error||detail?.paused||!metadata||!s||!Number.isFinite(checked)||now-checked< -1)return null;
  const pose=decodeObservation(s.native_pose_compact,s.feedback_age_ms+Math.max(0,now-checked),decode64);
  if(!pose||pose.zero_sha256!==metadata.sha)return null;
  pose.position_age_ms=pose.position_age_ms.map((age,i)=>pose.modes[i]===metadata.modes[i]?age:255);
  return {...pose,modes:metadata.modes.slice(),zeros:metadata.zeros.slice(),reference_pose:'q-zero',direction_corrections:IDS.map(id=>corrections[id]===-1?-1:1)};
 }
 const api={IDS,zeroMetadata,decodeObservation,posePacket};
 if(typeof module==='object'&&module.exports){module.exports=api;return;}
 const get=id=>document.getElementById(id),storageKey='xgoduck.pose-direction.v1';
 let metadata=null,latest=null,currentStatus=null,currentObservedAt=0,page='imu',corrections={};
 try{const raw=localStorage.getItem(storageKey);if(raw&&raw.length<2048){const saved=JSON.parse(raw);if(saved&&typeof saved==='object'&&!Array.isArray(saved)&&Object.entries(saved).every(([id,value])=>IDS.includes(Number(id))&&[1,-1].includes(value)))corrections=saved;}}catch{}
 function send(){
  const frame=get('imu-duck-frame');if(!frame?.contentWindow)return;
  const paused=document.hidden||page!=='imu',now=Date.now();
  const status=!paused&&latest?posePacket(latest,metadata,now,corrections,value=>atob(value)):null;
  frame.contentWindow.postMessage({type:'native-pose-state',version:1,mode:paused?'paused':status?'live':'stale',
   status,observedAtMs:now,reason:paused?'页面已暂停':!metadata?'等待保存机械零位':!status?'等待新鲜关节反馈':'编码器姿态 · 安装方向待核对'},location.origin);
 }
 function consume(detail){latest=detail;send();}
 function sendCurrent(){
  const frame=get('imu-duck-frame');if(!frame?.contentWindow)return;
  const paused=document.hidden||page!=='imu',sequence=currentStatus?.sequence_calibration;
  const held=sequence?.fresh===true&&sequence.holding_verified===true&&sequence.tolerance_ticks===57&&Array.isArray(sequence.held_ids)?sequence.held_ids:[];
  const rows=IDS.map(id=>{
   const row=currentStatus?.joints?.find(joint=>joint.id===id);
   return {id,current_raw:row?.current_raw??null,current_ma:row?.current_ma??null,current_age_ms:row?.current_age_ms??null,current_fresh:row?.current_fresh===true,
    current_source:row?.current_source,current_unit:row?.current_unit,
    holding_verified:held.includes(id)&&row?.fresh===true&&row?.torque_enabled===true};
  });
  frame.contentWindow.postMessage({type:'motor-current-state',version:1,mode:paused?'paused':currentStatus?'live':'stale',
   boot_id:currentStatus?.boot_id,rows,observedAtMs:currentObservedAt},location.origin);
 }
 function consumeCurrent(status){if(status){if(status!==currentStatus){currentStatus=status;currentObservedAt=Date.now();}sendCurrent();}}
 scope.addEventListener('duck:imu-status',event=>{if(scope.DuckIMUDatagramEnabled===true)consume(event.detail);});
 scope.addEventListener('duck:live-status',event=>{if(scope.DuckIMUDatagramEnabled!==true)consume(event.detail);});
 scope.addEventListener('duck:wireless-calibration',event=>{const state=event.detail?.status;if(state){metadata=zeroMetadata(state);send();consumeCurrent(state);}});
 scope.addEventListener('duck:sequence-status',event=>consumeCurrent(event.detail?.status));
 scope.addEventListener('duck:page',event=>{page=event.detail?.page||page;send();sendCurrent();});
 scope.addEventListener('message',event=>{const frame=get('imu-duck-frame');if(event.origin===location.origin&&event.source===frame?.contentWindow&&event.data?.type==='imu-model-ready'){send();sendCurrent();}});
 document.addEventListener('visibilitychange',()=>{if(document.hidden){latest=null;currentStatus=null;}send();sendCurrent();});
 const selector=get('pose-direction-joint'),button=get('pose-direction-flip'),note=get('pose-direction-note');
 function describeDirection(){const id=Number(selector?.value);if(note)note.textContent='ID '+id+'：'+(corrections[id]===-1?'已反向显示':'官方默认方向')+'；只改画面，不改变舵机和归位。';}
 selector?.addEventListener('change',describeDirection);
 button?.addEventListener('click',event=>{
  if(event?.isTrusted!==true)return;const id=Number(selector?.value);if(!IDS.slice(0,14).includes(id))return;
  corrections[id]=corrections[id]===-1?1:-1;
  try{localStorage.setItem(storageKey,JSON.stringify(corrections));}catch{}
  describeDirection();send();
 });
 describeDirection();
 // The iframe also uses per-position age; this timer only clears parent status.
 setInterval(()=>{if(latest&&!document.hidden&&page==='imu')send();},100);
})(typeof window==='object'?window:globalThis);
