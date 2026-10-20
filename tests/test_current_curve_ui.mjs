/* Current viewer lifecycle and canvas gaps against the real UI; no device or browser. */
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import vm from 'node:vm';
import {fileURLToPath} from 'node:url';

const root=path.resolve(path.dirname(fileURLToPath(import.meta.url)),'../pc');
const settle=()=>new Promise(resolve=>setImmediate(resolve));
const sha='a'.repeat(64);
function harness(file,{trained=false,sourceTransform=value=>value}={}){
 const nodes=new Map(),events={},intervals=[],calls=[],messages=[],timers=new Map(),draws=[];let nextTimer=0,currentData={ok:true,sample_hz:100,latest_recorded_unix_s:Date.now()/1000,samples:[]},currentError=false,currentDeferred=null;let clock=0,failStatus=false,delayRead=false,delayedRead=null,delayCommand=false,delayedCommand=null;
 class Element{
  constructor(tag='DIV'){this.tagName=tag.toUpperCase();this.children=[];this.dataset={};this.attributes=new Map();this.value='';this.options=[];this.textContent='';this.hidden=false;this.disabled=false;this.open=false;this.classList={add(){},remove(){},toggle(){}};this.contentWindow={postMessage:(...args)=>messages.push(args)};}
  set innerHTML(value){this._html=value;for(const match of value.matchAll(/<([a-z]+)[^>]*\bid="([^"]+)"[^>]*>/g)){const node=$(match[2]);node.tagName=match[1].toUpperCase();}
   if(value.includes('id="bench-forward-speed"')){$('bench-forward-speed').value='0.2';$('bench-forward-speed').options=[...value.matchAll(/<option value="([^"]+)"[^>]*>/g)].map(match=>({value:match[1],disabled:false}));}}
  get innerHTML(){return this._html||'';}
  append(...items){this.children.push(...items);}before(){}after(){}remove(){this.removed=true;}
  querySelector(selector){this.parts??=new Map();if(!this.parts.has(selector))this.parts.set(selector,new Element(selector));return this.parts.get(selector);}setAttribute(name,value){this.attributes.set(name,value);}removeAttribute(name){this.attributes.delete(name);}hasAttribute(name){return this.attributes.has(name);}
  showModal(){this.open=true;}close(){this.open=false;this.onclose?.();}click(){return this.onclick?.();}getContext(){return new Proxy({},{get:(_,name)=>(...args)=>draws.push({name,args}),set:()=>true});}
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
 const window={COMMISSION_TOKEN:'offline',parent:{postMessage:(...args)=>messages.push(args)},addEventListener:(name,fn)=>{(events[name]??=[]).push(fn);},dispatchEvent(){}};
 const context={console,document,window,location:{origin:'http://offline'},navigator:{sendBeacon(){throw Error('Unexpected unload write');}},
  Date,Number,Math,Map,Set,JSON,Error,Blob,URL,AbortSignal,AbortController,setTimeout:(fn,ms)=>{const id=++nextTimer;timers.set(id,{fn,ms});return id;},clearTimeout:id=>timers.delete(id),performance:{now:()=>clock},CustomEvent:class{constructor(name,value){this.name=name;this.detail=value?.detail;}},
  setInterval:(fn,delay)=>{intervals.push({fn,delay});},fetch:async(url,options={})=>{const body=options.body?JSON.parse(options.body):undefined;calls.push({url,body,options});if(url==='/api/bench/current-series'){if(currentError)throw Error('offline');if(currentDeferred===true)return new Promise(resolve=>currentDeferred=resolve);return {ok:true,json:async()=>structuredClone(currentData)};}
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
 return {$,state,calls,intervals,events,fire,key,messages,poll,document,draws,timers,current:value=>{currentData=value;},currentFail:value=>{currentError=value;},deferCurrent:()=>{currentDeferred=true;},releaseCurrent:async value=>{const resolve=currentDeferred;currentDeferred=null;resolve({ok:true,json:async()=>value});await settle();await settle();},runTimer:async()=>{const entry=[...timers].find(([,v])=>v.ms===500);if(entry){timers.delete(entry[0]);entry[1].fn();await settle();await settle();}},keyboardFactory:window.DuckKeyboard,advance:ms=>clock+=ms,failStatus:value=>failStatus=value,
  deferStatus:()=>{delayRead=true;},releaseStatus:async(error=false)=>{const item=delayedRead;assert.ok(item,'a deferred status request exists');delayedRead=null;if(error)item.reject(Error('old status failure'));else item.resolve({ok:true,json:async()=>item.snapshot});await settle();await settle();},
  deferCommand:()=>{delayCommand=true;},releaseCommand:async(error=false)=>{const item=delayedCommand;assert.ok(item,'a deferred command exists');delayedCommand=null;if(error)item.reject(Error('old command failure'));else item.resolve({ok:true,json:async()=>({ok:true})});await settle();await settle();}};
}


let checks=0;
const check=(condition,message)=>{assert.ok(condition,message);checks++;};
const h=harness('official-workbench.js');await settle();
const seriesCalls=()=>h.calls.filter(call=>call.url==='/api/bench/current-series');
check(seriesCalls().length===0,'closed dialog performs no current fetch');
const t=Date.now()/1000;
const sample=(offset,seq,mask=32767,sum=.351)=>({t:t+offset,seq,mask,sum_abs_a:sum,values:Array(15).fill(.0234)});
h.current({ok:true,sample_hz:100,latest_recorded_unix_s:t,samples:[sample(-.04,1),sample(-.03,2),sample(-.02,3,32766,null),sample(-.01,4),sample(0,5)]});
h.$('simple-current').click();await settle();await settle();
check(h.$('current-curve-dialog').open,'card opens native dialog');
check(seriesCalls().length===1,'one read starts on open');
check(seriesCalls().every(call=>call.body===undefined&&!call.options.method),'current viewer uses GET only');
check(h.$('current-curve-value').textContent==='0.351 A','actual latest amplitude shown');
check(h.$('current-curve-hz').textContent==='100.0 Hz','sensor sample rate kept distinct from poll rate');
check(h.$('current-curve-channel').children.length===15,'all fifteen single-servo channels offered');
check(h.draws.slice(h.draws.findLastIndex(call=>call.name==='beginPath')).filter(call=>call.name==='lineTo').length===2,'two curve line segments: missing axis breaks trace');
h.$('current-curve-channel').value='10';h.$('current-curve-channel').onchange();
check(h.$('current-curve-value').textContent==='0.023 A','single channel reads signed feedback');
h.$('current-curve-close').click();await settle();
check(!h.$('current-curve-dialog').open&&!h.timers.size,'close cancels polling');
const before=seriesCalls().length;await h.runTimer();check(seriesCalls().length===before,'closed timer never fetches');
h.$('simple-current').onkeydown({key:'Enter',preventDefault(){},stopPropagation(){}});await settle();await settle();
check(h.$('current-curve-dialog').open,'keyboard card activation opens viewer');
h.document.hidden=true;await h.events['document:visibilitychange']();await settle();
check(!h.timers.size&&/历史/.test(h.$('current-curve-state').textContent),'hidden page pauses and labels history');
h.document.hidden=false;await h.events['document:visibilitychange']();await settle();await settle();
check(h.timers.size===1,'visible dialog resumes a single poll loop');
h.currentFail(true);await h.runTimer();
check(/历史.*非实时/.test(h.$('current-curve-state').textContent),'network failure retains history and marks non-live');
h.currentFail(false);h.deferCurrent();await h.runTimer();const pendingCalls=seriesCalls().length;
await h.runTimer();check(seriesCalls().length===pendingCalls,'pending poll never overlaps');
h.$('current-curve-close').click();h.$('simple-current').click();await settle();await settle();
const value=h.$('current-curve-value').textContent;
await h.releaseCurrent({ok:true,sample_hz:1,samples:[sample(0,9,32767,99)]});
check(h.$('current-curve-value').textContent===value,'late old response cannot overwrite reopened session');
check(h.calls.every(call=>!call.body),'open, close, selection, visibility and polling send no control POST');
const html=fs.readFileSync(path.join(root,'index.html'),'utf8');
check(!html.match(/id="simple-current"[^\n]*current-log/),'top status card has no CSV link');
check(html.includes('aria-haspopup="dialog"')&&html.includes('id="current-curve-dialog"'),'dialog relationship in actual HTML');
check((html.match(/current-log\.csv/g)||[]).length===1,'CSV export exists once, inside viewer');
h.$('current-curve-close').click();
console.log(JSON.stringify({result:'PASS',checks,hardware_connected:false,browser_started:false,scope:'current viewer pure GET, gaps, channel selection, modal timer lifecycle and stale-response isolation'}));
