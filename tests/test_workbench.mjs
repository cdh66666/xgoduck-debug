/* Execute the real UI adapters against an offline DOM/API. No browser/hardware. */
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import vm from 'node:vm';
import {fileURLToPath} from 'node:url';

const root=path.resolve(path.dirname(fileURLToPath(import.meta.url)),'../pc');
const settle=()=>new Promise(resolve=>setImmediate(resolve));
const sha='a'.repeat(64);
function harness(file,{trained=false,standOnly=false,basic=false,sourceTransform=value=>value}={}){
 const nodes=new Map(),events={},intervals=[],calls=[],messages=[];let clock=0,failStatus=false,delayRead=false,delayedRead=null,delayCommand=false,delayedCommand=null,delayMode=false,delayedMode=null;
 class Element{
  constructor(tag='DIV'){this.tagName=tag.toUpperCase();this.children=[];this.dataset={};this.attributes=new Map();this.value='';this.options=[];this.textContent='';this.hidden=false;this.disabled=false;this.open=false;this.classList={add(){},remove(){},toggle(){}};this.contentWindow={postMessage:(...args)=>messages.push(args)};}
  set innerHTML(value){this._html=value;for(const match of value.matchAll(/<([a-z]+)[^>]*\bid="([^"]+)"[^>]*>/g)){const node=$(match[2]);node.tagName=match[1].toUpperCase();}
   if(value.includes('id="bench-forward-speed"')){$('bench-forward-speed').value='0.2';$('bench-forward-speed').options=[...value.matchAll(/<option value="([^"]+)"[^>]*>/g)].map(match=>({value:match[1],disabled:false}));}}
  get innerHTML(){return this._html||'';}
  append(...items){this.children.push(...items);}before(){}after(){}remove(){this.removed=true;}
  querySelector(selector){this.parts??=new Map();if(!this.parts.has(selector))this.parts.set(selector,new Element(selector));return this.parts.get(selector);}setAttribute(name,value){this.attributes.set(name,value);}removeAttribute(name){this.attributes.delete(name);}hasAttribute(name){return this.attributes.has(name);}
  show(){this.open=true;}showModal(){this.open=true;}close(){this.open=false;}click(){return this.onclick?.();}
  getContext(){return null;}
 }
 const $=id=>{if(!nodes.has(id))nodes.set(id,new Element());return nodes.get(id);};
 const tabs=['numbering','install','imu','models'].map(page=>{const node=new Element('button');node.dataset.page=page;return node;});
 const state={connected:true,runtime_basis_matches:true,revision:'offline-test',baseline:{reference:'9dc36787bc61',files_checked:28,local_sources_verified:true,
  walk_policy:{active:basic,sha256:basic?'57cdebf36e7fa23116270dec96f013a6a64511f37311d322443f5297bdc3d6db':undefined},basic_keyboard_policy:{active:basic,sha256:'57cdebf36e7fa23116270dec96f013a6a64511f37311d322443f5297bdc3d6db'},fast_policy:{active:trained,sha256:sha,run_id:trained?'fast-forward-specialist':null},navigation_policy:{active:trained},manual_pick:{active:trained}},test:null,
  runtime:{mode:'off',enabled:false,uptime_s:1,pc_host_epoch:1,full_robot_ready:true,servo_ids:Array(15).fill(10),imu_ok:true,feedback_hz:100,
   feedback_age_ms:1,mcu_command_age_us:1000,raw_pos:Array(15).fill(2048),zero_pos:Array(15).fill(2048),recovery:{phase:'walk',angle_deg:0},pick:{active:false},roulade:{active:false},
   manual_pick:{active:false,depth:0,target_depth:0,phase:0,max_phase:.18,returning:false,yaw_deferred:false},
   forward_specialist:{available:trained,requested:false,active:false,braking:false,brake_remaining_s:0,speed:.2,head_deferred:false},
   navigation_specialist:{available:trained,requested:false,active:false,braking:false,brake_remaining_s:0,head_deferred:false},
   fast_forward:{available:trained,requested:false,active:false,braking:false,brake_remaining_s:0,speed:.3,model:'fast-forward-specialist'}}};
 const document={body:new Element('body'),hidden:false,getElementById:$,createElement:tag=>new Element(tag),
  querySelectorAll:selector=>selector==='#workflow-tabs button'?tabs:selector==='#model-list button'?$('model-list').children:[],
  addEventListener:(name,fn)=>{events['document:'+name]=fn;}};
 const window={COMMISSION_TOKEN:'offline',XGO_RELEASE_PROFILE:basic?{schema:'xgoduck-basic-keyboard-publication-v2'}:standOnly?{schema:'xgoduck-stand-getup-publication-profile-v1'}:undefined,parent:{postMessage:(...args)=>messages.push(args)},addEventListener:(name,fn)=>{(events[name]??=[]).push(fn);},dispatchEvent(){}};
 const context={console,document,window,location:{origin:'http://offline'},navigator:{sendBeacon(){throw Error('Unexpected unload write');}},
  Date,Number,Math,Map,Set,JSON,Error,Blob,URL,AbortSignal,AbortController,performance:{now:()=>clock},CustomEvent:class{constructor(name,value){this.name=name;this.detail=value?.detail;}},
  setTimeout:()=>0,clearTimeout(){},setInterval:(fn,delay)=>{intervals.push({fn,delay});},fetch:async(url,options={})=>{const body=options.body?JSON.parse(options.body):undefined;calls.push({url,body,headers:options.headers,keepalive:options.keepalive});
   if(!body&&failStatus)throw Error('offline status failed');
   if(!body){state.runtime.uptime_s+=.05;if(delayRead){delayRead=false;const snapshot=structuredClone(state);return new Promise((resolve,reject)=>{delayedRead={resolve,reject,snapshot};});}}
   if(body&&url==='/api/mode'){state.runtime.mode=body.mode;state.runtime.enabled=body.mode==='policy'||body.mode==='hold';}
   if(body&&url==='/api/mode'&&delayMode){delayMode=false;return new Promise((resolve,reject)=>{delayedMode={resolve,reject};});}
   if(body&&url==='/api/command'&&delayCommand){delayCommand=false;return new Promise((resolve,reject)=>{delayedCommand={resolve,reject};});}
   return {ok:true,json:async()=>body?{}:structuredClone(state)};}};
 if(file==='official-workbench.js')vm.runInNewContext(fs.readFileSync(path.join(root,'assets/duck-keyboard.js'),'utf8'),context,{filename:'duck-keyboard.js'});
 vm.runInNewContext(sourceTransform(fs.readFileSync(path.join(root,'assets',file),'utf8')),context,{filename:file});
 const fire=async(name,event)=>{for(const fn of events[name]||[])await fn(event);await settle();};
 const key=async(type,key,repeat=false)=>fire(type,{key,repeat,code:key===' '?'Space':'Key'+key.toUpperCase(),target:{tagName:'BODY'},preventDefault(){}});
 const poll=async()=>{intervals.at(-1).fn();await settle();await settle();};
 return {$,tabs,state,calls,intervals,events,fire,key,messages,poll,hide:async value=>{document.hidden=value;events['document:visibilitychange']?.();await settle();},keyboardFactory:window.DuckKeyboard,advance:ms=>clock+=ms,failStatus:value=>failStatus=value,
  deferStatus:()=>{delayRead=true;},releaseStatus:async(error=false)=>{const item=delayedRead;assert.ok(item,'a deferred status request exists');delayedRead=null;if(error)item.reject(Error('old status failure'));else item.resolve({ok:true,json:async()=>item.snapshot});await settle();await settle();},
  deferMode:()=>{delayMode=true;},releaseMode:async()=>{const item=delayedMode;assert.ok(item,'a deferred mode request exists');delayedMode=null;item.resolve({ok:true,json:async()=>({ok:true})});await settle();await settle();},
  deferCommand:()=>{delayCommand=true;},releaseCommand:async(error=false)=>{const item=delayedCommand;assert.ok(item,'a deferred command exists');delayedCommand=null;if(error)item.reject(Error('old command failure'));else item.resolve({ok:true,json:async()=>({ok:true})});await settle();await settle();}};
}

let checks=0;
const check=(condition,message)=>{assert.ok(condition,message);checks++;};
const same=(actual,expected,message)=>{assert.deepEqual(actual,expected,message);checks++;};
const posts=h=>h.calls.filter(call=>call.body);
const packet=h=>h.calls.filter(call=>call.url==='/api/command').at(-1).body;
async function start(h){await settle();h.$('bench-policy').onclick();await h.$('bench-confirm').onclick();await h.poll();}
async function tap(h,key){await h.key('keydown',key);await h.key('keyup',key);}

const idle=harness('official-workbench.js');await settle();
check(posts(idle).length===0,'page load is read-only');
const staleCache=harness('official-workbench.js',{trained:true});await settle();
staleCache.state.runtime.feedback_age_ms=258;
await staleCache.poll();
check(staleCache.$('wl-home').disabled&&staleCache.$('bench-policy').disabled,'258ms cached feedback cannot start motion');
same(staleCache.$('simple-imu').querySelector('label').textContent,'IMU回报','hardware card describes the last report, not current freshness');
same(staleCache.$('feedback-timing').dataset.freshness,'stale','late data remains explicitly stale while hardware reports stay readable');
same(staleCache.$('feedback-age').textContent,'250 ms','actual age remains visible without hiding delayed data');
const agesWhilePending=harness('official-workbench.js',{trained:true});await settle();
agesWhilePending.$('bench-policy').onclick();
agesWhilePending.deferStatus();await agesWhilePending.poll();
agesWhilePending.advance(250);await agesWhilePending.poll();
check(agesWhilePending.$('wl-home').disabled,'browser elapsed time expires controls while status request is pending');
await agesWhilePending.$('bench-confirm').onclick();
check(posts(agesWhilePending).length===0,'confirmation rechecks age and cannot submit an old prepared start');
await agesWhilePending.releaseStatus();
check(agesWhilePending.$('wl-home').disabled,'a delayed old reply cannot renew the feedback age');
same(agesWhilePending.$('feedback-timing').dataset.freshness,'stale','delayed reply remains stale in status bar');
const queuedOldReply=harness('official-workbench.js',{trained:true});await settle();
Object.assign(queuedOldReply.state.runtime,{feedback_age_ms:8,native_feedback_age_ms:8,pc_cache_age_ms:0,pc_observed_unix_s:Date.now()/1000-.5});
await queuedOldReply.poll();
check(queuedOldReply.$('wl-home').disabled&&queuedOldReply.$('bench-policy').disabled,'existing observation clock includes time spent in a queued reply');
const queuedBacklog=harness('official-workbench.js',{trained:true});await settle();
Object.assign(queuedBacklog.state.runtime,{feedback_age_ms:103,native_feedback_age_ms:3,
 pc_delivery_age_lower_bound_ms:100,pc_cache_age_ms:0,pc_observed_unix_s:Date.now()/1000-.1});
await queuedBacklog.poll();
check(queuedBacklog.$('wl-home').disabled&&queuedBacklog.$('bench-policy').disabled,'delivery backlog remains counted while a reply waits in the browser queue');
same(queuedBacklog.$('feedback-timing').dataset.freshness,'stale','queued delayed state cannot be labeled as current IMU');
const stableReports=harness('official-workbench.js',{trained:true});await settle();
stableReports.state.wireless_transport_alive=true;
stableReports.state.runtime.servo_ids=[10,11,12,13,14,20,21,22,23,24,30,31,32,33,34];
await stableReports.poll();
const reportCards=()=>['simple-link','simple-servos','simple-imu','simple-motion'].map(id=>[stableReports.$(id).querySelector('strong').textContent,stableReports.$(id).dataset.state]);
const originalCards=reportCards();
stableReports.deferStatus();await stableReports.poll();stableReports.advance(2300);await stableReports.poll();
same(reportCards(),originalCards,'a browser scheduling pause is not misreported as a Wi-Fi disconnect');
check(stableReports.$('wl-home').disabled,'a browser scheduling pause still withdraws control permission');
await stableReports.releaseStatus();
for(const age of [149,151,80,180,99,150,70]){
 stableReports.state.runtime.feedback_age_ms=age;stableReports.state.runtime.feedback_hz=99.9;await stableReports.poll();
 same(reportCards(),originalCards,'short delivery jitter and fractional Hz variation do not invent hardware changes');
 same(stableReports.$('wl-home').disabled,age>=150,'every delayed frame still expires motion permission immediately');
}
stableReports.state.runtime.servo_ids.pop();stableReports.state.runtime.imu_ok=false;await stableReports.poll();
same(stableReports.$('simple-servos').querySelector('strong').textContent,'14 / 15','actual missing servo is shown immediately');
same(stableReports.$('simple-imu').querySelector('strong').textContent,'异常','actual IMU fault is shown immediately');
stableReports.state.runtime.joint_feedback_age_ms=Array(15).fill(201);
stableReports.state.runtime.pc_delivery_age_lower_bound_ms=100;stableReports.state.runtime.pc_cache_age_ms=90;await stableReports.poll();
same(stableReports.$('simple-servos').querySelector('strong').textContent,'15 / 15','one missed bus read with 11ms native joint age is not a lost servo; PC delivery age does not invent faults');
stableReports.state.runtime.joint_feedback_age_ms[14]=350;await stableReports.poll();
same(stableReports.$('simple-servos').querySelector('strong').textContent,'14 / 15','a genuinely 160ms missing joint is immediately reported');
stableReports.state.runtime.servo_current={available:false,supported:true,full_sample_age_ms:12,last_sum_abs_a:.312};await stableReports.poll();
same(stableReports.$('simple-current').querySelector('strong').textContent,'0.312 A','one skipped current slot retains the actual complete recent measurement');
stableReports.state.runtime.servo_current.full_sample_age_ms=180;await stableReports.poll();
same(stableReports.$('simple-current').querySelector('strong').textContent,'暂无完整采样','missing complete current samples do not silently remain current');
stableReports.state.connected=false;stableReports.state.status_stream={last_received_age_ms:2200};await stableReports.poll();
same(stableReports.$('simple-link').querySelector('strong').textContent,'反馈中断','a stalled transport is shown even while the SSH process is alive');
same(stableReports.$('feedback-timing').dataset.state,'interrupted','real interruption has one clear warning');
same(idle.$('bench-forward-speed').options.map(option=>option.value),['0.2'],'forward selector exposes only fixed 0.2');
idle.$('bench-forward-speed').value='0.4';idle.$('bench-forward-speed').onchange();
check(idle.$('bench-forward-speed').value==='0.2','tampered selector never changes ordinary speed');
for(const key of ['w',' ','k','u'])await tap(idle,key);
check(posts(idle).length===0,'unowned keyboard never sends motion');
check(/低头目标阶段 0%/.test(idle.$('bench-keyboard-readout').textContent),'depth stage is visible before movement');

const drive=harness('official-workbench.js',{trained:true});await settle();
drive.$('bench-policy').onclick();
check(/W用前进策略0.2 m\/s/.test(drive.$('bench-motion-description').textContent),'pure W new-model speed announced at 0.2');
check(/Shift\+W同策略0.3/.test(drive.$('bench-motion-description').textContent),'Shift+W same-model 0.3 announced separately');
check(/实物待验收/.test(drive.$('bench-motion-description').textContent),'physical result not claimed');
check(!/0\.1[–—-]0\.4|0\.4 m\/s/.test(drive.$('bench-motion-description').textContent),'no unsupported speed range');
check(posts(drive).length===0,'opening the announcement is read-only');
await drive.$('bench-confirm').onclick();await drive.poll();
same(packet(drive).twist,[0,0,0],'explicit start begins at zero');
check(packet(drive).manual_pick_depth===0&&!packet(drive).fast_forward&&!packet(drive).forward_specialist,'explicit start clears phase and both forward requests');
check(drive.$('bench-forward-speed').disabled,'ordinary speed control remains locked while owned');

await drive.key('keydown','w');same(packet(drive).twist,[.2,0,0],'pure W requests new PPO at 0.2');
check(packet(drive).forward_specialist===true&&packet(drive).fast_forward===false,'W alone selects specialist without acceleration');
check(/新模型 0.2/.test(drive.$('bench-forward-speed').options[0].textContent),'selector accurately labels active new model');
await drive.key('keydown','s');same(packet(drive).twist,[0,0,0],'W+S cancel');
check(!packet(drive).forward_specialist&&!packet(drive).fast_forward,'opposed W/S leave specialist');
await drive.key('keyup','w');same(packet(drive).twist,[-.3,0,0],'S requests admitted retained-official reverse 0.3 only');
check(packet(drive).navigation_specialist&&!packet(drive).forward_specialist&&!packet(drive).fast_forward,'S selects navigation only');
await drive.key('keyup','s');await drive.key('keydown','a');same(packet(drive).twist,[0,0,1],'A is admitted yaw 1');
check(packet(drive).navigation_specialist&&!packet(drive).forward_specialist,'A selects navigation, not old dead-zone model');
await drive.key('keydown','d');same(packet(drive).twist,[0,0,0],'A+D cancel');
await drive.key('keyup','a');same(packet(drive).twist,[0,0,-1],'D is admitted yaw -1');
await drive.key('keyup','d');same(packet(drive).twist,[0,0,0],'all motion key releases stop');

await drive.key('keydown','w');await drive.key('keydown','Shift');
same(packet(drive).twist,[.3,0,0],'Shift+W requests fixed specialist 0.3');
check(packet(drive).fast_forward===true&&packet(drive).forward_specialist===true,'both specialist and acceleration fields are explicit');
await drive.key('keydown','a');same(packet(drive).twist,[0,0,1],'Shift+W+A turns in place, no composite motion');
check(packet(drive).navigation_specialist&&packet(drive).fast_forward===false&&packet(drive).forward_specialist===false,'net turning selects navigation only');
await drive.key('keydown','d');same(packet(drive).twist,[.3,0,0],'A+D cancel, so still-held Shift+W is pure forward');
check(!packet(drive).navigation_specialist&&packet(drive).fast_forward&&packet(drive).forward_specialist,'no net turn uses only forward specialist');
await drive.key('keyup','a');await drive.key('keyup','d');
check(packet(drive).fast_forward===true&&packet(drive).forward_specialist===true,'released turns permit a still-held explicit Shift+W');
await drive.key('keyup','w');same(packet(drive).twist,[0,0,0],'release W immediately requests zero');
check(packet(drive).fast_forward===false&&packet(drive).forward_specialist===false,'release W clears specialist and fast requests');
await drive.key('keydown','s');same(packet(drive).twist,[-.3,0,0],'Shift+S remains admitted navigation reverse 0.3');
check(packet(drive).navigation_specialist&&!packet(drive).fast_forward,'reverse cannot select fast-forward');
await drive.key('keyup','s');await drive.key('keyup','Shift');
drive.state.runtime.forward_specialist.braking=true;drive.state.runtime.forward_specialist.brake_remaining_s=2.4;await drive.poll();
check(/零速度刹停中.*2.4 s/.test(drive.$('bench-keyboard-state').textContent),'backend specialist braking is displayed');
check(/前进模型期间头部指令已累计，回原版行走后应用/.test(drive.$('bench-keyboard-readout').textContent),'specialist braking defers head commands visibly');
drive.state.runtime.forward_specialist.braking=false;
for(const speed of [.2,.3]){
 drive.state.runtime.forward_specialist.active=true;drive.state.runtime.forward_specialist.speed=speed;await drive.poll();
 check(/前进模型期间头部指令已累计，回原版行走后应用/.test(drive.$('bench-keyboard-readout').textContent),`active ${speed} specialist marks head commands deferred`);
}
drive.state.runtime.forward_specialist.active=false;
drive.state.runtime.navigation_specialist.active=true;await drive.poll();
check(/导航模型期间头部指令已累计，回原站立后应用/.test(drive.$('bench-keyboard-readout').textContent),'navigation defers head commands too');
drive.state.runtime.navigation_specialist.braking=true;drive.state.runtime.navigation_specialist.brake_remaining_s=1.7;await drive.poll();
check(/导航退出.*1.7 s/.test(drive.$('bench-keyboard-state').textContent),'navigation braking is separately displayed');
drive.state.runtime.navigation_specialist.active=false;drive.state.runtime.navigation_specialist.braking=false;

await tap(drive,'i');same(packet(drive).head,[.05,-.05,0,0],'I at standing raises the head');
check(packet(drive).manual_pick_depth===0,'raised head does not request manual Pick');
await drive.intervals.find(item=>item.delay===50).fn();same(packet(drive).head,[.05,-.05,0,0],'raised head holds after I release');
await tap(drive,'i');same(packet(drive).head,[.1,-.1,0,0],'I raising accumulates');
await tap(drive,'k');same(packet(drive).head,[.05,-.05,0,0],'K first returns raised head toward HOME');
check(packet(drive).manual_pick_depth===0,'K cannot jump from raised head into squat');
await tap(drive,'k');same(packet(drive).head,[0,0,0,0],'K reaches HOME before manual Pick');
await tap(drive,'k');check(packet(drive).manual_pick_depth===.05,'K increments full-body Pick depth');
same(packet(drive).head,[0,0,0,0],'K never substitutes unverified head pitch');
check(/低头目标阶段 5%.*松键保持/.test(drive.$('bench-keyboard-readout').textContent),'requested depth and hold are visible');
await drive.intervals.find(item=>item.delay===50).fn();check(packet(drive).manual_pick_depth===.05,'K release holds the selected stage');
await drive.key('keydown','k');check(packet(drive).manual_pick_depth===.1,'new press accumulates');
await drive.key('keydown','k',true);check(packet(drive).manual_pick_depth===.1,'OS repeat is rate limited');
drive.advance(120);await drive.key('keydown','k',true);check(packet(drive).manual_pick_depth===.15,'long press slowly accumulates');
await drive.key('keydown','i');check(packet(drive).manual_pick_depth===.15,'opposite held phase keys freeze');
await drive.key('keyup','k');await drive.key('keyup','i');await tap(drive,'i');check(packet(drive).manual_pick_depth===.1,'I returns toward standing');
drive.state.runtime.manual_pick={active:true,depth:.08,target_depth:.1,phase:.0144,max_phase:.18,returning:false,yaw_deferred:true};await drive.poll();
check(/主控当前阶段 8%/.test(drive.$('bench-keyboard-readout').textContent),'backend phase is separate from local target');
check(/抬回后生效/.test(drive.$('bench-keyboard-readout').textContent),'deferred yaw is explained');
await tap(drive,'k');check(packet(drive).manual_pick_depth===.15,'manual Pick still accepts incremental K');
await tap(drive,'j');check(packet(drive).head[2]===.05,'J accumulates yaw for later backend application');
await tap(drive,'j');check(packet(drive).head[2]===.1,'yaw accumulates');
await tap(drive,'l');check(packet(drive).head[2]===.05,'L reverses yaw');
await tap(drive,'u');await tap(drive,'u');check(packet(drive).mouth===6,'mouth accumulates and holds while manual Pick is active');
await tap(drive,'o');check(packet(drive).mouth===3,'mouth closes incrementally');
check(!drive.$('bench-head-center').disabled,'center remains available while manual Pick active');
const autoPosts=posts(drive).length;await tap(drive,' ');
check(posts(drive).length===autoPosts,'manual Pick blocks Space without any POST');
check(drive.$('bench-pick').disabled&&drive.$('bench-roulade').disabled,'manual Pick blocks automatic model buttons');
await drive.$('bench-head-center').onclick();await settle();
check(packet(drive).manual_pick_depth===0,'head center clears manual phase');
same(packet(drive).head,[0,0,0,0],'center clears yaw and head');
check(packet(drive).mouth===3,'center preserves mouth selection');
drive.state.runtime.manual_pick={active:false,depth:0,target_depth:0,phase:0,max_phase:.18,returning:false,yaw_deferred:false};await drive.poll();

for(let i=0;i<30;i++){await tap(drive,'k');await tap(drive,'j');await tap(drive,'u');}
check(packet(drive).manual_pick_depth===1,'manual stage clamps at 100%');
same(packet(drive).head,[0,0,1,0],'only yaw accumulates to its validated bound');
check(packet(drive).mouth===30,'mouth clamps at 30 degrees');
for(let i=0;i<50;i++)await tap(drive,'i');
check(packet(drive).manual_pick_depth===0,'raising leaves the manual phase at zero');
same(packet(drive).head,[1,-1,1,0],'raising continues beyond HOME to its ordinary pitch clamp');
await drive.key('keydown','w');same(packet(drive).head,[1,-1,1,0],'walking preserves an ordinary raised-head command');await drive.key('keyup','w');
await drive.$('bench-head-center').onclick();await settle();
await tap(drive,'k');await drive.key('keydown','w');
check(packet(drive).manual_pick_depth===0,'WSAD clears the stored manual phase before movement');
await drive.key('keyup','w');await drive.intervals.find(item=>item.delay===50).fn();
check(packet(drive).manual_pick_depth===0,'released W never restores an old squat');
same(packet(drive).twist,[0,0,0],'released W never leaves residual walking');
await drive.key('keydown','w');await tap(drive,'k');
check(packet(drive).manual_pick_depth===0,'K cannot store a positive depth while W remains held');
await drive.key('keyup','w');await drive.intervals.find(item=>item.delay===50).fn();
check(packet(drive).manual_pick_depth===0,'releasing W after blocked K does not squat');
await drive.key('keydown','a');await tap(drive,'k');check(packet(drive).manual_pick_depth===0,'turning blocks positive depth too');
await tap(drive,'i');same(packet(drive).head,[.05,-.05,0,0],'ordinary I raising still accumulates while moving');
await drive.key('keyup','a');await drive.$('bench-head-center').onclick();await settle();
await tap(drive,'k');await drive.key('keydown','a');check(packet(drive).manual_pick_depth===0,'turning also clears old depth');await drive.key('keyup','a');
await drive.$('bench-head-center').onclick();await settle();

drive.state.runtime.pick.active=true;drive.state.runtime.mouth=12;await drive.poll();
await drive.intervals.find(item=>item.delay===50).fn();check(packet(drive).mouth===12,'automatic Pick owns mouth');
const blocked=posts(drive).length;await tap(drive,'k');await tap(drive,'u');await tap(drive,' ');
check(posts(drive).length===blocked,'automatic Pick ignores head/mouth/Space without side effects');
drive.state.runtime.pick.active=false;drive.state.runtime.recovery.phase='getup';await drive.poll();
await tap(drive,' ');check(posts(drive).length===blocked,'Space remains blocked while recovering');
drive.state.runtime.recovery.phase='walk';await drive.poll();
const rolls=drive.calls.filter(call=>call.url==='/api/roulade').length;
await drive.key('keydown',' ');await drive.key('keydown',' ',true);await drive.key('keydown',' ',true);
check(drive.calls.filter(call=>call.url==='/api/roulade').length===rolls+1,'one held Space sends one model request');
await drive.key('keyup',' ');await tap(drive,' ');
check(drive.calls.filter(call=>call.url==='/api/roulade').length===rolls+1,'pending rollout cannot be repeated');
drive.state.runtime.roulade.active=true;await drive.poll();drive.state.runtime.roulade.active=false;await drive.poll();
await tap(drive,' ');check(drive.calls.filter(call=>call.url==='/api/roulade').length===rolls+2,'a later explicit Space works after completion');
await drive.key('keydown','Escape');await settle();
check(drive.calls.filter(call=>call.url==='/api/mode').at(-1).body.mode==='off','Escape requests OFF');
const stopped=posts(drive).length;await drive.key('keydown','Escape',true);await tap(drive,'w');await drive.intervals.find(item=>item.delay===50).fn();
check(posts(drive).length===stopped,'Stop removes ownership and suppresses repeat/resume');

const unavailable=harness('official-workbench.js');await start(unavailable);await unavailable.key('keydown','w');await unavailable.key('keydown','Shift');
check(posts(unavailable).length===0,'old Host never receives mode-policy or unsupported new command fields');
check(unavailable.$('bench-policy').disabled,'upgrade capability guard disables keyboard start');
check(/键盘控制升级待部署/.test(unavailable.$('bench-keyboard-state').textContent),'missing upgrade is clearly displayed');
for(const missing of ['fast_policy','navigation_policy','manual_pick']){
 const partial=harness('official-workbench.js',{trained:true});await settle();partial.state.baseline[missing].active=false;await partial.poll();
 await start(partial);check(posts(partial).length===0,`missing ${missing} blocks prepare and forced confirmation`);
}
const race=harness('official-workbench.js',{trained:true});await settle();race.$('bench-policy').onclick();race.state.baseline.navigation_policy.active=false;await race.poll();await race.$('bench-confirm').onclick();
check(posts(race).length===0,'capability loss between announcement and confirmation blocks mode request');
unavailable.$('wl-home').onclick();await unavailable.$('bench-confirm').onclick();await unavailable.poll();
check(unavailable.calls.some(call=>call.url==='/api/mode'&&call.body?.mode==='hold'),'legacy hold remains explicitly available without keyboard deployment');
same(Object.keys(packet(unavailable)).sort(),['head','mouth','twist'],'hold uses only the legacy API schema');
await unavailable.$('wl-stop').onclick();check(unavailable.calls.filter(call=>call.url==='/api/mode').at(-1).body.mode==='off','legacy STOP remains available');
const engine=unavailable.keyboardFactory.create();
for(const key of ['s','a','d']){
 engine.reset();engine.keydown({key,code:'Key'+key.toUpperCase()});const value=engine.packet({fastAllowed:true,navAllowed:false});
 same(Array.from(value.twist),[0,0,0],`unavailable navigation ${key} emits no nonzero fallback`);check(!value.navigation_specialist,`unavailable navigation ${key} flag stays false`);
}
engine.reset();for(const key of ['w','s','a'])engine.keydown({key});
same(Array.from(engine.packet({fastAllowed:true,navAllowed:true}).twist),[0,0,1],'net AD wins even when W/S oppose');
engine.keydown({key:'d'});same(Array.from(engine.packet({fastAllowed:true,navAllowed:true}).twist),[0,0,0],'all opposing directions cancel without composite inputs');

const actionGate=harness('official-workbench.js',{trained:true});await start(actionGate);
await actionGate.key('keydown','w');
let actionGatePosts=posts(actionGate).length;
await tap(actionGate,' ');check(posts(actionGate).length===actionGatePosts,'Space while walking keys held produces no POST');
check(/先松开走动键，停稳后再按空格/.test(actionGate.$('wl-result').textContent),'invalid moving Space tells the user how to retry explicitly');
await actionGate.intervals.find(item=>item.delay===50).fn();check(posts(actionGate).length===actionGatePosts+1,'invalid Space preserves ownership and heartbeat');
await actionGate.key('keydown','s');actionGatePosts=posts(actionGate).length;
await tap(actionGate,' ');check(posts(actionGate).length===actionGatePosts,'opposed walking keys still must be released before Space');
await actionGate.key('keyup','w');await actionGate.key('keyup','s');
for(const model of ['forward_specialist','navigation_specialist']){
 for(const flag of ['active','braking']){
  actionGate.state.runtime[model][flag]=true;await actionGate.poll();actionGatePosts=posts(actionGate).length;
  await tap(actionGate,' ');check(posts(actionGate).length===actionGatePosts,`${model} ${flag} blocks Space without POST`);
  check(actionGate.$('bench-pick').disabled&&actionGate.$('bench-roulade').disabled,`${model} ${flag} disables both automatic-action buttons`);
  actionGate.$('bench-pick').onclick();await actionGate.$('bench-confirm').onclick();check(posts(actionGate).length===actionGatePosts,`${model} ${flag} also blocks forced automatic Pick`);
  await actionGate.intervals.find(item=>item.delay===50).fn();check(posts(actionGate).length===actionGatePosts+1,`${model} ${flag} rejection never relinquishes command ownership`);
  actionGate.state.runtime[model][flag]=false;
 }
}
await actionGate.poll();actionGate.$('bench-pick').onclick();actionGate.state.runtime.navigation_specialist.braking=true;await actionGate.poll();actionGatePosts=posts(actionGate).length;
await actionGate.$('bench-confirm').onclick();check(posts(actionGate).length===actionGatePosts,'new braking between Pick announcement and confirmation is read-only rejection');
await actionGate.intervals.find(item=>item.delay===50).fn();check(posts(actionGate).length===actionGatePosts+1,'late Pick rejection preserves ownership too');
actionGate.state.runtime.navigation_specialist.braking=false;await actionGate.poll();
actionGatePosts=actionGate.calls.filter(call=>call.url==='/api/roulade').length;await tap(actionGate,' ');
check(actionGate.calls.filter(call=>call.url==='/api/roulade').length===actionGatePosts+1,'after explicit release and completed braking, Space works once without queued actions');

const lost=harness('official-workbench.js',{trained:true});await start(lost);await tap(lost,'k');await tap(lost,'u');await lost.key('keydown','w');
lost.failStatus(true);await lost.poll();
check(/待读取/.test(lost.$('simple-link').querySelector('strong').textContent),'poll failure clears link proof');
check(/最近一次主控回报/.test(lost.$('simple-servos').querySelector('small').textContent),'poll failure retains only explicitly historical servo report');
check(lost.$('feedback-timing').dataset.state==='interrupted','poll failure clearly marks the retained report as interrupted');
check(lost.$('simple-motion').querySelector('label').textContent==='扭矩回报'&&lost.$('diag-torque').textContent==='未知','cached torque report is not current torque proof');
check(lost.$('bench-policy').disabled,'poll failure disables motion start');
const lostPosts=posts(lost).length;lost.failStatus(false);await lost.poll();await lost.intervals.find(item=>item.delay===50).fn();await tap(lost,'w');await tap(lost,'u');
check(posts(lost).length===lostPosts,'reconnect never restores old movement, phase, or mouth');
check(/低头目标阶段 0%.*嘴 0°/.test(lost.$('bench-keyboard-readout').textContent),'lost ownership resets local phase and mouth');

const recovery=harness('official-workbench.js',{trained:true});await start(recovery);await tap(recovery,'u');await tap(recovery,'k');await recovery.key('keydown','w');
recovery.state.runtime.recovery.phase='getup';await recovery.poll();
const recoveringPosts=posts(recovery).length;await recovery.intervals.find(item=>item.delay===50).fn();
check(posts(recovery).length===recoveringPosts+1,'recovery preserves page ownership and heartbeat');
same(packet(recovery).twist,[0,0,0],'recovery clears movement');same(packet(recovery).head,[0,0,0,0],'recovery clears head intent');
check(packet(recovery).manual_pick_depth===0&&!packet(recovery).forward_specialist&&!packet(recovery).navigation_specialist&&!packet(recovery).fast_forward,'recovery sends no stale specialist intent');
check(packet(recovery).mouth===3,'recovery keeps mouth');
for(const key of ['w','k','u',' '])await tap(recovery,key);
recovery.state.runtime.recovery.phase='walk';await recovery.poll();await recovery.intervals.find(item=>item.delay===50).fn();
same(packet(recovery).twist,[0,0,0],'return to walk never restores old direction');
await recovery.key('keydown','w',true);await recovery.intervals.find(item=>item.delay===50).fn();same(packet(recovery).twist,[0,0,0],'OS autorepeat cannot restore discarded key intent');
await recovery.key('keyup','w');await recovery.key('keydown','w');same(packet(recovery).twist,[.2,0,0],'fresh press after recovery is accepted');
const supply=harness('official-workbench.js');await settle();supply.state.runtime.servo_power={available:true,voltage_v:7.4,raw:74,servo_id:10,age_ms:100};await supply.poll();
check(supply.$('battery-percent').textContent==='7.4 V'&&supply.$('simple-battery').dataset.state==='reported','fresh servo supply displays volts without inventing capacity');
supply.state.runtime.servo_power.age_ms=7000;supply.state.runtime.servo_power.available=false;await supply.poll();
check(supply.$('battery-percent').textContent==='7.4 V · 上次'&&supply.$('simple-battery').dataset.state==='pending','old voltage is visibly historical');
supply.state.runtime.servo_power=null;await supply.poll();check(supply.$('battery-percent').textContent==='待读取','missing voltage stays unknown');
check(posts(supply).length===0&&supply.calls.every(call=>call.url.split('?')[0]==='/api/bench/status'),'voltage tile reuses telemetry without a new read API or write');

// Reproduce the old ownership decision with a request-time OFF snapshot. This
// mutation restores only the old two poll decisions; it is not a historical file.
const legacy=harness('official-workbench.js',{trained:true,sourceTransform:source=>source
 .replace("const requestStarted=performance.now(),snapshot=await api('/api/bench/status'+cursor);if(generation!==actionGeneration)return;","const requestStarted=performance.now(),snapshot=await api('/api/bench/status'+cursor);")
 .replace("!sessionPending&&(s?.mode==='off'||s?.mode==='shadow')","(s?.mode==='off'||s?.mode==='shadow')")});
await settle();legacy.deferStatus();await legacy.poll();await start(legacy);await legacy.releaseStatus();
let beforeHeartbeat=posts(legacy).length;await legacy.intervals.find(item=>item.delay===50).fn();
same(posts(legacy).length,beforeHeartbeat,'old OFF ownership decision reproducibly stops control heartbeats');

const startup=harness('official-workbench.js',{trained:true});await settle();
startup.deferStatus();await startup.poll();await start(startup);await startup.releaseStatus();
beforeHeartbeat=posts(startup).length;await startup.intervals.find(item=>item.delay===50).fn();
check(posts(startup).length===beforeHeartbeat+1,'pre-start delayed OFF cannot cancel new zero heartbeats');
startup.state.runtime.mode='off';startup.state.runtime.enabled=false;await startup.poll();
await startup.key('keydown','w');await tap(startup,'k');await startup.key('keydown',' ');
await startup.intervals.find(item=>item.delay===50).fn();
same(packet(startup).twist,[0,0,0],'post-ACK cached OFF keeps only neutral heartbeat');
check(packet(startup).manual_pick_depth===0&&!packet(startup).forward_specialist&&!packet(startup).navigation_specialist,'pending start rejects positive specialist/depth intent');
check(/启动确认中/.test(startup.$('bench-keyboard-state').textContent),'pending confirmation is visible');
check(!startup.calls.some(call=>call.url==='/api/roulade'),'Space cannot start an action during confirmation');
startup.state.runtime.mode='policy';startup.state.runtime.enabled=true;await startup.poll();
await startup.key('keydown','w',true);await startup.intervals.find(item=>item.delay===50).fn();
same(packet(startup).twist,[0,0,0],'a held key from before confirmation cannot resume by repeat');
await startup.key('keyup','w');await startup.key('keydown','w');same(packet(startup).twist,[.2,0,0],'new matching snapshot enables a fresh key press');
await startup.key('keyup','w');startup.state.runtime.mode='shadow';startup.state.runtime.enabled=false;
startup.state.runtime.last_error='web heartbeat lost; disarmed';await startup.poll();
check(/心跳中断/.test(startup.$('wl-result').textContent),'real heartbeat loss is explained to the operator');
beforeHeartbeat=posts(startup).length;await startup.intervals.find(item=>item.delay===50).fn();
same(posts(startup).length,beforeHeartbeat,'confirmed session still relinquishes immediately on real shadow');

const lateFailure=harness('official-workbench.js',{trained:true});await start(lateFailure);
lateFailure.deferStatus();await lateFailure.poll();await lateFailure.$('wl-stop').onclick();
await start(lateFailure);await lateFailure.releaseStatus(true);beforeHeartbeat=posts(lateFailure).length;
await lateFailure.intervals.find(item=>item.delay===50).fn();
check(posts(lateFailure).length===beforeHeartbeat+1,'failed GET from previous session cannot cancel restarted zero heartbeat');
await lateFailure.poll();await lateFailure.key('keydown','w');same(packet(lateFailure).twist,[.2,0,0],'new session survives old GET exception');

const lateCommand=harness('official-workbench.js',{trained:true});await start(lateCommand);
lateCommand.deferCommand();const inFlight=lateCommand.intervals.find(item=>item.delay===50).fn();await settle();
await lateCommand.$('wl-stop').onclick();await start(lateCommand);await lateCommand.releaseCommand(true);await inFlight;
beforeHeartbeat=posts(lateCommand).length;await lateCommand.intervals.find(item=>item.delay===50).fn();
check(posts(lateCommand).length===beforeHeartbeat+1,'old command exception cannot revoke new session');

const deadline=harness('official-workbench.js',{trained:true});await settle();
deadline.$('bench-policy').onclick();await deadline.$('bench-confirm').onclick();
deadline.state.runtime.mode='off';deadline.state.runtime.enabled=false;await deadline.poll();
deadline.advance(3501);await deadline.poll();
check(!deadline.calls.some(call=>call.url==='/api/mode'&&call.body?.mode==='off'),'unconfirmed policy startup relinquishes ownership without unloading');
beforeHeartbeat=posts(deadline).length;deadline.state.runtime.mode='policy';deadline.state.runtime.enabled=true;await deadline.poll();
await deadline.key('keydown','w');await deadline.intervals.find(item=>item.delay===50).fn();
same(posts(deadline).length,beforeHeartbeat,'late matching mode after timeout never restores ownership');

const cancelled=harness('official-workbench.js',{trained:true});await settle();
cancelled.$('bench-policy').onclick();await cancelled.$('bench-confirm').onclick();
cancelled.deferStatus();await cancelled.poll();await cancelled.$('wl-stop').onclick();await cancelled.releaseStatus();
beforeHeartbeat=posts(cancelled).length;await cancelled.key('keydown','w');await cancelled.intervals.find(item=>item.delay===50).fn();
same(posts(cancelled).length,beforeHeartbeat,'STOP while confirmation pending survives late policy snapshot');

const legacyHold=harness('official-workbench.js');await settle();legacyHold.$('wl-home').onclick();await legacyHold.$('bench-confirm').onclick();await legacyHold.poll();
check(legacyHold.$('bench-forward-speed').disabled===true,'ordinary hold remains owned without keyboard capabilities');
await legacyHold.intervals.find(item=>item.delay===50).fn();same(packet(legacyHold),{twist:[0,0,0],head:[0,0,0,0],mouth:0},'legacy hold confirms with original neutral packet');
const exited=harness('official-workbench.js',{trained:true});await start(exited);
exited.state.runtime.mode='shadow';exited.state.runtime.enabled=false;exited.state.runtime.last_error='';await exited.poll();
check(/已退出上力控制.*键盘已暂停/.test(exited.$('wl-result').textContent),'mode exit without an error replaces the old startup-success note');
const publication=harness('official-workbench.js',{standOnly:true});await settle();
check(posts(publication).length===0,'publication profile loads read-only');
publication.$('bench-policy').onclick();
check(/零速站立/.test(publication.$('bench-motion-description').textContent),'zero-speed policy prepares with fast/nav/manual capability false');
check(/零速站立/.test(publication.$('bench-motion-description').textContent),'publication zero-speed action is announced');
await publication.$('bench-confirm').onclick();await publication.poll();
same(packet(publication).twist,[0,0,0],'publication policy starts at zero');
check(publication.$('bench-pick').disabled&&publication.$('bench-roulade').disabled,'unaccepted action buttons remain visible and disabled');
for(const key of ['w','s','a','d','Shift',' ','k'])await tap(publication,key);
await publication.intervals.find(item=>item.delay===50).fn();
check(publication.calls.filter(call=>call.url==='/api/command').every(call=>call.body.twist.every(v=>v===0)&&call.body.manual_pick_depth===0),'blocked movement and deep K preserve zero heartbeats');
check(!publication.calls.some(call=>call.url==='/api/pick'||call.url==='/api/roulade'),'blocked Space never sends a model action');
check(publication.state.runtime.mode==='policy','blocked K never cancels standing');
await tap(publication,'j');await tap(publication,'u');
check(packet(publication).head[2]!==0&&packet(publication).mouth>0,'ordinary head yaw and mouth remain available');
await tap(publication,'i');check(packet(publication).head[0]>0,'ordinary raised-head command remains available');
await tap(publication,'k');same(packet(publication).head.slice(0,2),[0,0],'K returns raised head to HOME');
await tap(publication,'k');check(packet(publication).manual_pick_depth===0,'K cannot cross HOME into deep Pick');
await publication.$('wl-stop').onclick();check(publication.state.runtime.mode==='off','publication Stop still unloads');
const simple=harness('official-workbench.js',{standOnly:true});await settle();
check(fs.readFileSync(path.join(root,'assets/official-workbench.js'),'utf8').includes('>控制</button>'), 'control entry has the short label');
const beforeSimple=posts(simple).length;
simple.$('bench-test-panel').hidden=true;simple.$('sequence-panel').hidden=true;
simple.$('bench-tests-toggle').onclick();
check(!simple.$('bench-test-panel').hidden&&simple.$('sequence-panel').hidden,'control opens independently');
simple.$('sequence-toggle').onclick();
check(simple.$('bench-test-panel').hidden&&!simple.$('sequence-panel').hidden,'calibration replaces control panel');
simple.$('calibration-panel-close').onclick();
check(simple.$('sequence-panel').hidden,'calibration close remains available');
check(posts(simple).length===beforeSimple,'panel opening and closing send no device writes');
const html=fs.readFileSync(path.join(root,'index.html'),'utf8');
check(/<div id="bench-more-tests"[^>]*hidden[^>]*inert/.test(html)&&!html.includes('<summary>更多测试</summary>'),'legacy test DOM is hidden and inert without a visible entry');
check(/<div id="calibration-details"[^>]*hidden[^>]*inert/.test(html)&&!html.includes('<summary>高级 / 详情</summary>'),'legacy calibration details hidden and inert without an entry');
check(html.includes('href="/assets/calibration-tray.html"')&&html.includes('href="/downloads/XGO-Duck-Calibration-Tray-R7.stl"'),'approved R7 preview and existing download linked');
check(html.includes('托盘内只校准，取出放地面后再站立。'),'tray boundary remains visible without starting a model');
check(!simple.$('calibration-tray-frame').hasAttribute('src'),'tray preview starts unloaded');
simple.$('calibration-tray-open').onclick({preventDefault(){}});
check(simple.$('calibration-tray-dialog').open&&simple.$('calibration-tray-frame').hasAttribute('src'),'tray preview opens lazily in the same page');
check(posts(simple).length===beforeSimple,'opening tray preview sends no device writes');
simple.$('calibration-tray-close').onclick();
check(!simple.$('calibration-tray-dialog').open&&!simple.$('calibration-tray-frame').hasAttribute('src'),'closing tray preview unloads its frame');
const trayOwned=harness('official-workbench.js',{standOnly:true});await start(trayOwned);
trayOwned.$('calibration-tray-open').onclick({preventDefault(){}});
const beforeTrayKey=posts(trayOwned).length;await tap(trayOwned,'j');
check(posts(trayOwned).length===beforeTrayKey,'head keys cannot pass through tray modal');
await trayOwned.key('keydown','Escape');
check(!trayOwned.$('calibration-tray-dialog').open&&trayOwned.state.runtime.mode==='off','Escape closes tray preview and preserves original stop/unload');
const beforeWrong=posts(trayOwned).length;
await trayOwned.fire('message',{origin:'http://wrong',source:trayOwned.$('calibration-tray-frame').contentWindow,data:{type:'duck-tray-escape'}});
check(posts(trayOwned).length===beforeWrong,'wrong-origin tray message never acts');
check(/<details id="calibration-single-details"[^>]*>/.test(html)&&!/<details id="calibration-single-details"[^>]*\bopen\b/.test(html),'single-servo controls collapsed by default');
check(/id="calibration-single-details"[\s\S]*id="sequence-joints"[\s\S]*id="sequence-capture"[\s\S]*<\/details>/.test(html),'all fifteen choices and save action retained in the single-servo section');
check(/id="calibration-details"[\s\S]*id="calibration-restore"/.test(html),'legacy exit node retained only for handler compatibility');
check(/<dialog id="bench-test-panel"/.test(html),'control uses a native dialog');
check(/<dialog id="calibration-flow-dialog"/.test(html),'calibration uses a native dialog');
check(/id="wl-home-state"[^>]*hidden/.test(html),'changing long motion status stays outside the main model view');
check(html.indexOf('<main class="simple-home">')<html.indexOf('class="live-strip"')&&html.indexOf('class="live-strip"')<html.indexOf('id="workflow-install"'),'six live status cards remain at the top of the main page');
check(/id="current-curve-open"[^>]*hidden/.test(html),'old header current button is hidden while its handler ID remains compatible');
const guideSource=fs.readFileSync(path.join(root,'assets/official-workbench.js'),'utf8');
for(const key of ['w','a','s','d','shift','space','i','k','j','l','u','o'])check(guideSource.includes('data-key="'+key+'"'),'visual keyboard includes '+key);
simple.$('bench-tests-toggle').onclick();check(simple.$('bench-test-panel').open,'control entry opens the nonmodal panel without motion');
simple.$('bench-test-close').onclick();check(!simple.$('bench-test-panel').open,'control close leaves no dialog behind');
check(posts(simple).length===beforeSimple,'native control open and close remain read-only');
check(guideSource.includes('if(!panel.open)panel.show();')&&!guideSource.includes('if(!panel.open)panel.showModal();'),'control panel opens nonmodal while separate action confirmation remains modal');
check(guideSource.includes('document.body.dataset.controlOpen=String(visible)')&&guideSource.includes("document.body.dataset.controlOpen='false'"),'opening and closing release reserved model space');
check(/id="simple-current"[^\n]*<strong>[^<]*<\/strong><small hidden>/.test(html),'current number and click target retained while only the small sampling note is hidden');
const midAck=harness('official-workbench.js',{trained:true});await settle();
midAck.deferCommand();midAck.$('bench-policy').onclick();const pendingAck=midAck.$('bench-confirm').onclick();await settle();
await midAck.poll();check(/主控已确认启动/.test(midAck.$('wl-result').textContent),'fresh confirmation can arrive before the first heartbeat HTTP reply');
await midAck.releaseCommand();await pendingAck;
check(/主控已确认启动/.test(midAck.$('wl-result').textContent),'late command completion cannot overwrite confirmed state with a waiting message');
// A cached/incomplete startup snapshot cannot cancel the explicit zero heartbeat.
const startupGap=harness('official-workbench.js',{trained:true});await settle();
startupGap.$('bench-policy').onclick();await startupGap.$('bench-confirm').onclick();
startupGap.state.runtime.full_robot_ready=false;await startupGap.poll();
let beforeGapPosts=posts(startupGap).length;await startupGap.intervals.find(item=>item.delay===50).fn();
check(posts(startupGap).length===beforeGapPosts+1,'startup missing feedback keeps zero heartbeat until bounded confirmation');
same(packet(startupGap).twist,[0,0,0],'startup gap cannot command movement');
startupGap.state.runtime.full_robot_ready=true;await startupGap.poll();
check(/主控已确认/.test(startupGap.$('wl-result').textContent),'new fresh enabled state confirms startup after one missing snapshot');
startupGap.state.connected=false;await startupGap.poll();startupGap.advance(437);
beforeGapPosts=posts(startupGap).length;await startupGap.intervals.find(item=>item.delay===50).fn();
check(posts(startupGap).length===beforeGapPosts+1,'437ms state transport gap does not stop the command heartbeat');
same(packet(startupGap).twist,[0,0,0],'transport gap keeps zero velocity');
startupGap.state.connected=true;await startupGap.poll();
check(/反馈已恢复/.test(startupGap.$('wl-result').textContent),'fresh feedback recovers with explicit release of pressed keys');
startupGap.state.connected=false;await startupGap.poll();startupGap.advance(801);await startupGap.poll();
check(!startupGap.calls.some(row=>row.url==='/api/mode'&&row.body?.mode==='off'),'persistent policy feedback gap never sends automatic OFF');
beforeGapPosts=posts(startupGap).length;startupGap.state.connected=true;await startupGap.poll();await tap(startupGap,'w');await startupGap.intervals.find(item=>item.delay===50).fn();
same(posts(startupGap).length,beforeGapPosts,'returning status never reacquires policy ownership after a gap');
const neverConfirmed=harness('official-workbench.js',{trained:true});await settle();neverConfirmed.$('bench-policy').onclick();await neverConfirmed.$('bench-confirm').onclick();
neverConfirmed.state.connected=false;neverConfirmed.advance(3600);await neverConfirmed.poll();
check(!neverConfirmed.calls.some(row=>row.url==='/api/mode'&&row.body?.mode==='off'),'startup deadline without snapshots keeps local policy independent');
for(const event of ['hidden','pagehide']){
 const h=harness('official-workbench.js',{basic:true});await start(h);await h.key('keydown','w');const at=h.calls.length;
 if(event==='hidden')await h.hide(true);else await h.fire('pagehide');
 const out=h.calls.slice(at);check(out.some(row=>row.url==='/api/command'&&row.body?.twist.every(v=>v===0)),event+' sends a neutral intent');
 check(!out.some(row=>row.url==='/api/mode'),event+' never unloads policy');
 await h.hide(false);await h.poll();await tap(h,'w');await h.intervals.find(item=>item.delay===50).fn();
 same(h.calls.filter(row=>row.url==='/api/command').length,h.calls.slice(0,at).filter(row=>row.url==='/api/command').length+1,event+' requires explicitly reacquiring control');
 const reading=harness('official-workbench.js',{basic:true});await settle();await reading.hide(true);await reading.fire('pagehide');same(posts(reading).length,0,event+' remains read-only without ownership');
}
const spaceFocus=harness('official-workbench.js',{standOnly:true});await settle();spaceFocus.$('bench-test-panel').hidden=true;spaceFocus.$('bench-tests-toggle').onclick();
let downPrevented=false,upPrevented=false;
await spaceFocus.fire('keydown',{key:' ',code:'Space',target:{tagName:'BUTTON'},preventDefault(){downPrevented=true;}});
await spaceFocus.fire('keyup',{key:' ',code:'Space',target:{tagName:'BUTTON'},preventDefault(){upPrevented=true;}});
check(downPrevented&&upPrevented,'Space cannot activate the focused close button');
check(spaceFocus.$('bench-test-panel').open&&posts(spaceFocus).length===0,'inactive Space neither closes controls nor starts motion');
// A modal or another workflow cannot retain an old physical walking command.
const contexts={
 models:h=>h.tabs.find(tab=>tab.dataset.page==='models').click(),
 calibration:h=>{h.$('sequence-panel').hidden=true;h.$('sequence-toggle').click();},
 diagnostics:h=>h.$('diagnostics-open').click(),
 current:h=>h.$('simple-current').click(),
 tray:h=>h.$('calibration-tray-open').onclick({preventDefault(){}}),
};
for(const [name,open] of Object.entries(contexts)){
 const h=harness('official-workbench.js',{basic:true,trained:true});await start(h);await h.key('keydown','w');
 same(packet(h).twist,[.2,0,0],name+' begins with the admitted live command');
 const at=h.calls.length;open(h);await settle();await settle();
 await h.intervals.find(item=>item.delay===50).fn();
 check(h.calls.slice(at).some(call=>call.url==='/api/mode'&&call.body?.mode==='off'),name+' ends the physical control session');
 check(!h.calls.slice(at).some(call=>call.url==='/api/command'&&call.body.twist.some(v=>v!==0)),name+' cannot continue an old walking packet');
}
// The robot may accept a start before its HTTP response reaches this page.
const delayedLoss=harness('official-workbench.js',{basic:true});
// Do not await the unresolved start promise until its response is released.
await settle();delayedLoss.deferMode();await delayedLoss.$('bench-policy').onclick();const lossStart=delayedLoss.$('bench-confirm').onclick();await settle();
delayedLoss.failStatus(true);await delayedLoss.poll();delayedLoss.failStatus(false);await delayedLoss.releaseMode();await lossStart;
same(delayedLoss.calls.filter(row=>row.url==='/api/mode').map(row=>row.body.mode),['policy'],'a late start ACK after GET failure cannot automatically unload policy');
await delayedLoss.poll();const lossAt=delayedLoss.calls.length;await tap(delayedLoss,'w');await delayedLoss.intervals.find(item=>item.delay===50).fn();
check(!delayedLoss.calls.slice(lossAt).some(row=>row.url==='/api/command'),'a late ACK after GET failure cannot reacquire control');
const abandoned={hidden:h=>h.hide(true),pagehide:h=>h.fire('pagehide'),...contexts};
for(const [name,leave] of Object.entries(abandoned)){
 const h=harness('official-workbench.js',{basic:true});await settle();h.deferMode();await h.$('bench-policy').onclick();const result=h.$('bench-confirm').onclick();await settle();
 await leave(h);await h.releaseMode();await result;
 check(!h.calls.some(row=>row.url==='/api/mode'&&row.body?.mode==='off'),name+' abandons an unacknowledged policy start without automatic OFF');
 await h.hide(false);h.tabs.find(tab=>tab.dataset.page==='imu').click();
 for(const id of ['diagnostics-dialog','current-curve-dialog','calibration-tray-dialog','calibration-flow-dialog'])h.$(id).close();
 await h.poll();const at=h.calls.length;await tap(h,'w');await h.intervals.find(item=>item.delay===50).fn();
 check(!h.calls.slice(at).some(row=>row.url==='/api/command'),name+' late ACK cannot reacquire ownership after returning to live view');
 if(name==='hidden'||name==='pagehide'){
  const neutral=h.calls.find(row=>row.url==='/api/command'&&row.keepalive);
  check(neutral?.headers['X-Token']==='offline'&&neutral.body.twist.every(v=>v===0),name+' neutral keepalive is authenticated');
 }
}
for(const method of ['button','Escape']){
 const h=harness('official-workbench.js',{basic:true});await settle();h.deferMode();await h.$('bench-policy').onclick();const result=h.$('bench-confirm').onclick();await settle();
 if(method==='button')await h.$('wl-stop').onclick();else await h.key('keydown','Escape');
 await h.$('bench-policy').onclick();await h.$('bench-confirm').onclick();
 same(h.calls.filter(row=>row.url==='/api/mode'&&row.body?.mode==='policy').length,1,method+' cannot queue a second start before the first reply');
 await h.releaseMode();await result;
 same(h.calls.filter(row=>row.url==='/api/mode').at(-1).body.mode,'off',method+' still wins after a late policy ACK');
 await h.poll();const at=h.calls.length;await tap(h,'w');await h.intervals.find(item=>item.delay===50).fn();
 check(!h.calls.slice(at).some(row=>row.url==='/api/command'),method+' late ACK never restores movement');
}
for(const context of ['hidden','models']){
 const h=harness('official-workbench.js');await settle();h.deferMode();await h.$('wl-home').onclick();const result=h.$('bench-confirm').onclick();await settle();
 if(context==='hidden')await h.hide(true);else h.tabs.find(tab=>tab.dataset.page==='models').click();
 await h.releaseMode();await result;
 same(h.calls.filter(row=>row.url==='/api/mode').at(-1).body.mode,'off',context+' abandoned hold still ends OFF after its late acknowledgement');
}
const torquePending=harness('official-workbench.js');await settle();
torquePending.state.runtime.mode='off';torquePending.state.runtime.enabled=true;await torquePending.poll();
same(torquePending.$('simple-motion').querySelector('strong').textContent,'卸力中','OFF intent is not disabled feedback');
torquePending.state.runtime.enabled=false;await torquePending.poll();
same(torquePending.$('simple-motion').querySelector('strong').textContent,'已卸力','fresh disabled feedback confirms torque release');
same(torquePending.$('wl-home').textContent,'校准回中','executed JS preserves the calibration-only button name');
torquePending.state.runtime.mode='servo_debug';torquePending.state.runtime.enabled=false;
torquePending.state.runtime.factory_calibration={active:true,held_mask:1};await torquePending.poll();
same(torquePending.$('simple-motion').querySelector('strong').textContent,'校准 / 维护','normal-loop disabled flag cannot prove individual calibration axes are unloaded');
same(torquePending.$('diag-torque').textContent,'校准 / 维护','diagnostics retain the same individual-axis maintenance state');
same(torquePending.$('simple-motion').querySelector('small').textContent,'逐轴状态见校准','maintenance detail never claims all axes are released');
console.log(JSON.stringify({result:'PASS',checks,hardware_connected:false,browser_started:false,scope:'original 186 publication and legacy behavior checks plus simplified panel and read-only tray modal; portable published clone files'}));
