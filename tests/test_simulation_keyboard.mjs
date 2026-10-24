import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import vm from 'node:vm';
const source=fs.readFileSync(new URL('../pc/assets/model-training.js',import.meta.url),'utf8').replace(/^import .*;\r?\n/gm,'').replace('initDone=init().catch(failure);','initDone=Promise.resolve();');
const keyboard=fs.readFileSync(new URL('../pc/assets/duck-keyboard.js',import.meta.url),'utf8');
const settle=()=>new Promise(r=>setImmediate(r));
async function harness(model='walk'){
 const nodes=new Map(),events={},calls=[],pending=[];let focused=true,defer=false,seq=0;
 class Element{constructor(){this.dataset={};this.textContent='';this.hidden=false;this.disabled=false;}setAttribute(){}focus(){focused=true;}setPointerCapture(){}}
 const $=id=>{if(!nodes.has(id))nodes.set(id,new Element());return nodes.get(id);};
 const buttons=['w','s','a','d','shift'].map(key=>{const b=new Element();b.dataset.motion=key;return b;});
 const backend={loaded:true,paused:true,policyEnabled:false,body_poses:[],command:[0,0,0],modelSha:'a'.repeat(64),activeModel:model};
 const document={body:$('body'),hidden:false,hasFocus:()=>focused,getElementById:$,querySelectorAll:()=>buttons,addEventListener:(n,f)=>(events[n]??=[]).push(f)};
 const window={addEventListener:(n,f)=>(events['window:'+n]??=[]).push(f),parent:{postMessage(){}}};
 const context={window,document,parent:window.parent,location:{search:'?model=official-'+model,origin:'http://offline'},URLSearchParams,performance,Date,AbortSignal,console,setTimeout,clearTimeout,
  THREE:{Vector3:class{},Vector2:class{},Raycaster:class{},ArrowHelper:class{}},fetch:async(url,options)=>{
   assert.equal(url,'/api/simulation','keyboard never addresses hardware');const p=JSON.parse(options.body);calls.push(p);
   if(defer&&p.op==='command'){defer=false;await new Promise(r=>pending.push(r));}
   if(p.op==='command')backend.command=p.twist;
   if(p.op==='policy'){backend.policyEnabled=p.enabled;backend.paused=!p.enabled;}
   if(p.op==='pause'){backend.paused=p.paused;if(p.paused)backend.command=[0,0,0];}
   if(p.op==='reset')Object.assign(backend,{paused:true,policyEnabled:false,command:[0,0,0]});
   return {ok:true,json:async()=>({ok:true,state:{...backend,seq:++seq}})};
  }};
 vm.createContext(context);vm.runInContext(keyboard,context);vm.runInContext(source,context);
 vm.runInContext("session='only-local';token='test';frameView=()=>{};bindKeyboard();",context);
 context.initial={...backend,seq:0};vm.runInContext('applyState(initial)',context);
 async function fire(type,key,code=key,repeat=false){for(const fn of events[type]||[])await fn({key,code,repeat,target:{tagName:'BODY'},preventDefault(){}});await settle();await settle();}
 return {$,buttons,calls,backend,context,fire,press:(key,code)=>fire('keydown',key,code),release:(key,code)=>fire('keyup',key,code),start:()=>fire('keydown',' ','Space'),
  blur:async()=>{focused=false;await fire('window:blur','');},focus:()=>{focused=true;},defer:()=>{defer=true;},finish:async()=>{pending.shift()();await settle();await settle();},
  suspend:async()=>{document.hidden=true;await fire('visibilitychange','');},resume:async()=>{document.hidden=false;focused=true;await fire('visibilitychange','');},
  twist:()=>backend.command,eval:code=>vm.runInContext(code,context)};
}
test('actual simulation adapter accepts arrow/WASD aliases, acceleration, opposites and turn priority',async()=>{
 const h=await harness();await h.start();assert.equal(h.backend.paused,false);
 for(const [key,code,expected] of [['ArrowUp','ArrowUp',[.2,0,0]],['ArrowDown','ArrowDown',[-.3,0,0]],['ArrowLeft','ArrowLeft',[0,0,1]],['ArrowRight','ArrowRight',[0,0,-1]]]){
  await h.press(key,code);assert.deepEqual(h.twist(),expected);await h.release(key,code);assert.deepEqual(h.twist(),[0,0,0]);
 }
 await h.press('w','KeyW');await h.press('Shift','ShiftLeft');assert.deepEqual(h.twist(),[.3,0,0]);
 await h.press('ArrowUp','ArrowUp');await h.release('w','KeyW');assert.deepEqual(h.twist(),[.3,0,0],'releasing one alias does not release the other');
 await h.press('s','KeyS');assert.deepEqual(h.twist(),[0,0,0]);await h.press('a','KeyA');assert.deepEqual(h.twist(),[0,0,1]);
 await h.press('Escape','Escape');assert.deepEqual(h.twist(),[0,0,0]);
});
test('pause, reset, blur and hidden pages discard held input and never resume it',async()=>{
 const h=await harness();await h.press('w','KeyW');await h.start();assert.deepEqual(h.twist(),[0,0,0]);
 await h.press('w','KeyW');await h.blur();assert.deepEqual(h.twist(),[0,0,0]);h.focus();
 await h.fire('keydown','w','KeyW',true);assert.deepEqual(h.twist(),[0,0,0]);
 await h.press('w','KeyW');await h.start();assert.equal(h.backend.paused,true);assert.deepEqual(h.twist(),[0,0,0]);
 await h.start();assert.deepEqual(h.twist(),[0,0,0]);await h.press('w','KeyW');await h.suspend();assert.equal(h.backend.paused,true);
 await h.resume();assert.equal(h.backend.paused,true);await h.start();await h.press('r','KeyR');assert.equal(h.backend.policyEnabled,false);assert.deepEqual(h.twist(),[0,0,0]);
});
test('slow request coalesces intermediate directions and still delivers key release',async()=>{
 const h=await harness();await h.start();const start=h.calls.length;h.defer();await h.press('w','KeyW');
 await h.press('a','KeyA');await h.release('a','KeyA');await h.release('w','KeyW');await h.finish();
 assert.deepEqual(h.calls.slice(start).filter(c=>c.op==='command').map(c=>c.twist),[[.2,0,0],[0,0,0]]);
 await h.press('w','KeyW');await h.release('w','KeyW');assert.deepEqual(h.twist(),[0,0,0],'a duplicate zero does not wedge future sends');
});
test('pointer cancellation releases virtual movement; other model cards do not expose walking keys',async()=>{
 const h=await harness();await h.start();const b=h.buttons.find(b=>b.dataset.motion==='w');
 b.onpointerdown({button:0,pointerId:7,preventDefault(){}});await settle();assert.deepEqual(h.twist(),[.2,0,0]);
 b.onpointercancel({pointerId:7});await settle();assert.deepEqual(h.twist(),[0,0,0]);
 const other=await harness('getup');await other.start();const count=other.calls.length;await other.press('ArrowUp');assert.equal(other.calls.length,count);assert.equal(other.$('virtual-controls').hidden,true);
});
