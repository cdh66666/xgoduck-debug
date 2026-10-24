/* Execute the real UI adapters against an offline DOM/API. No browser/hardware. */
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import vm from 'node:vm';
import {fileURLToPath} from 'node:url';

const root=path.resolve(path.dirname(fileURLToPath(import.meta.url)),'../pc');
const settle=()=>new Promise(resolve=>setImmediate(resolve));
const sha='a'.repeat(64);
function harness(file,{trained=false,standOnly=false,sourceTransform=value=>value}={}){
 const nodes=new Map(),events={},intervals=[],calls=[],messages=[];let calibrationState=null,restoreError=false,restoreDelayReads=0;let clock=0,failStatus=false,delayRead=false,delayedRead=null,delayCommand=false,delayedCommand=null;
 class Element{
  constructor(tag='DIV'){this.tagName=tag.toUpperCase();this.children=[];this.dataset={};this.attributes=new Map();this.value='';this.options=[];this.textContent='';this.hidden=false;this.disabled=false;this.open=false;this.classList={add(){},remove(){},toggle(){}};this.contentWindow={postMessage:(...args)=>messages.push(args)};}
  set innerHTML(value){this._html=value;for(const match of value.matchAll(/<([a-z]+)[^>]*\bid="([^"]+)"[^>]*>/g)){const node=$(match[2]);node.tagName=match[1].toUpperCase();}
   if(value.includes('id="bench-forward-speed"')){$('bench-forward-speed').value='0.2';$('bench-forward-speed').options=[...value.matchAll(/<option value="([^"]+)"[^>]*>/g)].map(match=>({value:match[1],disabled:false}));}}
  get innerHTML(){return this._html||'';}
  set id(value){this._id=value;nodes.set(value,this);}get id(){return this._id;}
  append(...items){this.children.push(...items);}before(){}after(){}remove(){this.removed=true;}
  querySelector(selector){this.parts??=new Map();if(!this.parts.has(selector))this.parts.set(selector,new Element(selector));return this.parts.get(selector);}setAttribute(name,value){this.attributes.set(name,value);}removeAttribute(name){this.attributes.delete(name);}hasAttribute(name){return this.attributes.has(name);}
  showModal(){this.open=true;}close(){this.open=false;}click(){return this.onclick?.();}
 }
 const $=id=>{if(!nodes.has(id))nodes.set(id,new Element());return nodes.get(id);};
 const state={connected:true,runtime_basis_matches:true,revision:'offline-test',baseline:{reference:'9dc36787bc61',files_checked:28,local_sources_verified:true,
  walk_policy:{active:false},fast_policy:{active:trained,sha256:sha,run_id:trained?'fast-forward-specialist':null},navigation_policy:{active:trained},manual_pick:{active:trained}},test:null,
  runtime:{mode:'off',enabled:false,uptime_s:1,pc_host_epoch:1,full_robot_ready:true,servo_ids:Array(15).fill(10),imu_ok:true,feedback_hz:100,
   feedback_age_ms:1,mcu_command_age_us:1000,raw_pos:Array(15).fill(2048),zero_pos:Array(15).fill(2048),recovery:{phase:'walk',angle_deg:0},pick:{active:false},roulade:{active:false},
   manual_pick:{active:false,depth:0,target_depth:0,phase:0,max_phase:.18,returning:false,yaw_deferred:false},
   forward_specialist:{available:trained,requested:false,active:false,braking:false,brake_remaining_s:0,speed:.2,head_deferred:false},
   navigation_specialist:{available:trained,requested:false,active:false,braking:false,brake_remaining_s:0,head_deferred:false},
   fast_forward:{available:trained,requested:false,active:false,braking:false,brake_remaining_s:0,speed:.3,model:'fast-forward-specialist'}}};
 const document={body:new Element('body'),hidden:false,getElementById:$,createElement:tag=>new Element(tag),
  querySelectorAll:selector=>selector==='#model-list button'?$('model-list').children:[],
  addEventListener:(name,fn)=>{events['document:'+name]=fn;}};
 const window={COMMISSION_TOKEN:'offline',XGO_RELEASE_PROFILE:standOnly?{schema:'xgoduck-stand-getup-publication-profile-v1'}:undefined,parent:{postMessage:(...args)=>messages.push(args)},addEventListener:(name,fn)=>{(events[name]??=[]).push(fn);},dispatchEvent(){}};
 window.DuckCalibration={paint(){},select(){},snapshot:()=>calibrationState,isMaintenance:()=>!!calibrationState?.maintenance?.active,stop:async()=>{}};
 const context={setTimeout:(fn,ms)=>{clock+=ms;return setImmediate(fn);},console,document,window,location:{origin:'http://offline'},navigator:{sendBeacon(){throw Error('Unexpected unload write');}},
  Date,Number,Math,Map,Set,JSON,Error,Blob,URL,AbortSignal,performance:{now:()=>clock},CustomEvent:class{constructor(name,value){this.name=name;this.detail=value?.detail;}},
  setInterval:(fn,delay)=>{intervals.push({fn,delay});},fetch:async(url,options={})=>{const body=options.body?JSON.parse(options.body):undefined;calls.push({url,body});if(url==='/api/bench/calibration/action'){if(restoreError)throw Error('退出校准请求失败');calibrationState.calibration.active=true;return {ok:true,json:async()=>structuredClone(calibrationState.calibration)};}if(url==='/api/bench/calibration'){if(restoreDelayReads>0)restoreDelayReads--;else{calibrationState.maintenance.active=false;calibrationState.maintenance.phase='official';calibrationState.calibration.active=false;}return {ok:true,json:async()=>structuredClone(calibrationState)};}
   if(!body&&failStatus)throw Error('offline status failed');
   if(!body){state.runtime.uptime_s+=.05;if(delayRead){delayRead=false;const snapshot=structuredClone(state);return new Promise((resolve,reject)=>{delayedRead={resolve,reject,snapshot};});}}
   if(body&&url==='/api/mode'){state.runtime.mode=body.mode;state.runtime.enabled=body.mode==='policy'||body.mode==='hold';}
   if(body&&url==='/api/command'&&delayCommand){delayCommand=false;return new Promise((resolve,reject)=>{delayedCommand={resolve,reject};});}
   return {ok:true,json:async()=>body?{}:structuredClone(state)};}};
 if(file==='official-workbench.js')vm.runInNewContext(fs.readFileSync(path.join(root,'assets/duck-keyboard.js'),'utf8'),context,{filename:'duck-keyboard.js'});
 vm.runInNewContext(sourceTransform(fs.readFileSync(path.join(root,'assets',file),'utf8')),context,{filename:file});
 const fire=async(name,event)=>{for(const fn of events[name]||[])await fn(event);await settle();};
 const key=async(type,key,repeat=false)=>fire(type,{key,repeat,code:key===' '?'Space':'Key'+key.toUpperCase(),target:{tagName:'BODY'},preventDefault(){}});
 const poll=async()=>{intervals.at(-1).fn();await settle();await settle();};
 return {$,state,calls,intervals,events,fire,key,messages,poll,setCalibration:value=>{calibrationState=value;},restoreFails:value=>{restoreError=value;},restoreWait:value=>{restoreDelayReads=value;},keyboardFactory:window.DuckKeyboard,advance:ms=>clock+=ms,failStatus:value=>failStatus=value,
  deferStatus:()=>{delayRead=true;},releaseStatus:async(error=false)=>{const item=delayedRead;assert.ok(item,'a deferred status request exists');delayedRead=null;if(error)item.reject(Error('old status failure'));else item.resolve({ok:true,json:async()=>item.snapshot});await settle();await settle();},
  deferCommand:()=>{delayCommand=true;},releaseCommand:async(error=false)=>{const item=delayedCommand;assert.ok(item,'a deferred command exists');delayedCommand=null;if(error)item.reject(Error('old command failure'));else item.resolve({ok:true,json:async()=>({ok:true})});await settle();await settle();}};
}


let checks=0;const check=(condition,message)=>{assert.ok(condition,message);checks++;};
const posts=h=>h.calls.filter(call=>call.body);
const maintained=()=>({maintenance:{active:true,phase:'maintenance',integrated:true},calibration:{active:false,error:''}});
const idle=harness('official-workbench.js',{standOnly:true});await settle();
idle.state.runtime.angles=Array(15).fill(.35);await idle.poll();
idle.$('wl-home').onclick();
check(posts(idle).length===0,'fifteen finite fresh angles at threshold cause no writes');
check(idle.$('wl-result').textContent==='已在校准中位，无需回正','no-op explicitly reported');
idle.setCalibration(maintained());await idle.poll();
check(!idle.$('wl-home').disabled,'successful maintenance hold does not disable HOME');
idle.$('wl-home').onclick();check(posts(idle).length===0,'HOME at middle never exits maintenance');
for(const invalid of [Array(14).fill(0),[null,...Array(14).fill(0)],['0',...Array(14).fill(0)],[NaN,...Array(14).fill(0)]]){
 idle.state.runtime.angles=invalid;await idle.poll();idle.$('bench-motion-description').textContent='';idle.$('wl-home').onclick();
 check(idle.$('bench-motion-dialog').open&&idle.$('bench-motion-description').textContent.includes('回到校准中位并保持'),'missing or nonnumeric angles require confirmation instead of becoming a zero no-op');idle.$('bench-cancel').onclick();
}
idle.$('bench-motion-description').textContent='';idle.state.runtime.angles=Array(15).fill(0);idle.state.runtime.feedback_age_ms=150;await idle.poll();idle.$('wl-home').onclick();
check(idle.$('bench-motion-description').textContent===''&&posts(idle).length===0,'150ms feedback never qualifies for fresh HOME');
const offset=harness('official-workbench.js',{standOnly:true});await settle();offset.state.runtime.angles=[.3501,...Array(14).fill(0)];offset.setCalibration(maintained());await offset.poll();
offset.$('wl-home').onclick();
check(posts(offset).length===0&&offset.$('bench-motion-description').textContent.length>0,'deviation opens only confirmation');
check(offset.$('bench-motion-description').textContent.includes('回到校准中位并保持'),'HOME never promises the old standing pose');
await offset.$('bench-confirm').onclick();
check(offset.calls.filter(call=>call.url==='/api/bench/calibration/action'&&call.body.action==='restore').length===1,'one explicit restore after confirmation');
const restoreIndex=offset.calls.findIndex(call=>call.url==='/api/bench/calibration/action'),readIndex=offset.calls.findIndex(call=>call.url==='/api/bench/calibration'),holdIndex=offset.calls.findIndex(call=>call.url==='/api/mode'&&call.body.mode==='hold');
check(restoreIndex<readIndex&&readIndex<holdIndex,'verified maintenance exit precedes hold');
check(offset.state.runtime.mode==='hold','accepted HOME owns original hold mode');
const timeout=harness('official-workbench.js',{standOnly:true});await settle();timeout.state.runtime.angles=Array(15).fill(1);timeout.setCalibration(maintained());timeout.restoreWait(Infinity);await timeout.poll();timeout.$('wl-home').onclick();await timeout.$('bench-confirm').onclick();
check(!timeout.calls.some(call=>call.url==='/api/mode'&&call.body.mode==='hold'),'unconfirmed exit never starts HOME');
check(timeout.calls.filter(call=>call.url==='/api/bench/calibration/action').length===1,'exit timeout never replays restore');
check(/5秒/.test(timeout.$('wl-result').textContent),'bounded exit timeout is visible');
const failed=harness('official-workbench.js',{standOnly:true});await settle();failed.state.runtime.angles=Array(15).fill(2);failed.setCalibration(maintained());failed.restoreFails(true);await failed.poll();failed.$('wl-home').onclick();await failed.$('bench-confirm').onclick();
check(!failed.calls.some(call=>call.url==='/api/mode'&&call.body.mode==='hold'),'restore error never starts HOME');
check(failed.calls.filter(call=>call.url==='/api/bench/calibration/action').length===1,'restore error never replays a POST');
const ownedHome=harness('official-workbench.js',{standOnly:true});await settle();
const maintaining=harness('official-workbench.js',{standOnly:true});await settle();
maintaining.state.runtime.mode='hold';maintaining.state.runtime.policy_targets=Array(15).fill(0);maintaining.state.runtime.angles=Array(15).fill(.8);await maintaining.poll();
maintaining.$('wl-home').onclick();check(posts(maintaining).length===0,'already holding the identical calibrated target never restarts HOME despite servo following error');
check(maintaining.$('wl-result').textContent==='正在保持校准中位，无需重复回正','holding is not mislabeled as measured arrival');
ownedHome.$('bench-policy').onclick();await ownedHome.$('bench-confirm').onclick();await ownedHome.poll();
await ownedHome.key('keydown','j');await ownedHome.key('keyup','j');
ownedHome.state.runtime.angles=Array(15).fill(.1);await ownedHome.poll();
const ownedBefore=posts(ownedHome).length,headBefore=ownedHome.calls.filter(call=>call.url==='/api/command').at(-1).body.head;
ownedHome.$('wl-home').onclick();check(posts(ownedHome).length===ownedBefore,'already-middle HOME does not write during an existing policy session');
await ownedHome.intervals.find(item=>item.delay===50).fn();
check(posts(ownedHome).length===ownedBefore+1,'already-middle HOME preserves the current heartbeat owner');
assert.deepEqual(ownedHome.calls.filter(call=>call.url==='/api/command').at(-1).body.head,headBefore);checks++;
console.log(JSON.stringify({result:'PASS',checks,hardware_connected:false,scope:'strict calibrated-middle no-op and explicit confirmed exit-before-HOME; actual UI adapter with offline API'}));
