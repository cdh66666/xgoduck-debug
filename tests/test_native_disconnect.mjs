/* Execute the complete native page script; all fetch and input are in memory. */
import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import vm from 'node:vm';

const source=fs.readFileSync(new URL('../robot-app/assets/app.js',import.meta.url),'utf8');
const settle=()=>new Promise(resolve=>setImmediate(resolve));
const plain=value=>JSON.parse(JSON.stringify(value));
function harness(){
 const nodes=new Map(),events={},intervals=[],calls=[];let failStatus=false,failCommand=false,deferCommand=false,pendingCommand=null;
 class Element{
  constructor(){this.value='0';this.textContent='';this.style={};this.dataset={};this.firstElementChild={style:{},offsetWidth:40};this.clientWidth=200;}
  insertAdjacentHTML(){}getBoundingClientRect(){return {left:0,top:0,width:200,height:200};}
  click(){return this.onclick?.();}hasPointerCapture(){return false;}setPointerCapture(){}releasePointerCapture(){}
 }
 const $=id=>{if(!nodes.has(id))nodes.set(id,new Element());return nodes.get(id);};
 const buttons=['off','hold','policy','shadow'].map(mode=>{const e=new Element();e.dataset.mode=mode;return e;});
 const state={mode:'policy',enabled:true,control_ready:true,feedback_hz:100,inference_hz:50,
  servo_ids:[10,11,12,13,14,20,21,22,23,24,30,31,32,33,34],imu_ok:true,last_error:'',bridge_baud:2000000,
  feedback_age_ms:2,recovery:{active:false},acc:[0,0,-1],gyro:[0,0,0],gravity:[0,0,-1]};
 const document={hidden:false,getElementById:$,querySelectorAll:selector=>selector==='[data-mode]'?buttons:buttons.filter(b=>['hold','policy'].includes(b.dataset.mode)),addEventListener:(name,fn)=>events[name]=fn};
 const context={document,window:{addEventListener:(name,fn)=>events[name]=fn},
  setInterval:(fn,ms)=>intervals.push({fn,ms}),navigator:{sendBeacon:(url,blob)=>calls.push({url,blob})},Blob,JSON,Set,Math,
  fetch:async(url,options={})=>{
   const body=options.body?JSON.parse(options.body):undefined;calls.push({url,body});
   if(!body&&failStatus)throw Error('status response lost');
   if(body&&url==='/api/command'&&failCommand)throw Error('command response lost');
   if(body&&url==='/api/mode'){state.mode=body.mode;state.enabled=['policy','hold'].includes(body.mode);}
   if(body&&url==='/api/command'&&deferCommand){deferCommand=false;return new Promise(resolve=>pendingCommand=resolve);}
   return {ok:true,json:async()=>body?{}:state};
  }};
 vm.runInNewContext(source+'\nglobalThis.snapshot=()=>({keys:[...keys],motion:[...motion],inputPaused,neutralPending,statusHealthy});',context);
 return {$,calls,events,snapshot:()=>plain(context.snapshot()),
  failStatus:value=>failStatus=value,failCommand:value=>failCommand=value,
  deferCommand:()=>deferCommand=true,releaseCommand:async()=>{assert.ok(pendingCommand);pendingCommand({ok:true,json:async()=>({})});pendingCommand=null;await settle();await settle();},
  key:async(key,repeat=false)=>{events.keydown({key,repeat,code:key===' '?'Space':'Key'+key.toUpperCase(),preventDefault(){}});await settle();await settle();},
  keyup:async key=>{events.keyup({key});await settle();},
  tick:async ms=>{await intervals.find(item=>item.ms===ms).fn();await settle();await settle();},
  hide:async hidden=>{document.hidden=hidden;events.visibilitychange();await settle();await settle();}};
}
const nonzero=row=>row.url==='/api/command'&&row.body?.twist.some(v=>v!==0);

for(const channel of ['status','command'])test('native '+channel+' failure clears old intent and requires a fresh explicit input',async()=>{
 const h=harness();await h.key('w');assert.deepEqual(h.calls.filter(nonzero).at(-1).body.twist,[.2,0,0]);
 if(channel==='status'){h.failStatus(true);await h.tick(250);h.failStatus(false);await h.tick(250);}
 else{h.failCommand(true);await h.tick(50);h.failCommand(false);}
 assert.deepEqual(h.snapshot().keys,[]);assert.deepEqual(h.snapshot().motion,[0,0,0]);assert.equal(h.snapshot().inputPaused,true);
 const at=h.calls.length;await h.key('w',true);await h.tick(50);await h.tick(250);await h.tick(50);
 assert.equal(h.calls.slice(at).some(nonzero),false,'status recovery/timer/repeated held key cannot resume W');
 await h.keyup('w');await h.key('w');assert.deepEqual(h.calls.filter(nonzero).at(-1).body.twist,[.2,0,0]);
 assert.equal(h.calls.some(row=>row.url==='/api/mode'),false,'communication failure does not send OFF');
});

test('native hide/pagehide neutralize the complete script and returning does not resume W',async()=>{
 const h=harness();await h.key('w');await h.hide(true);h.events.pagehide();
 const beacon=h.calls.find(row=>row.blob);assert.equal(beacon.url,'/api/command');
 assert.deepEqual(JSON.parse(await beacon.blob.text()).twist,[0,0,0]);
 assert.equal(h.snapshot().inputPaused,true);const at=h.calls.length;
 await h.hide(false);await h.tick(250);await h.key('w',true);await h.tick(50);
 assert.equal(h.calls.slice(at).some(nonzero),false);assert.equal(h.calls.some(row=>row.url==='/api/mode'),false);
});

test('native old joystick pointer cannot resume after a failed status request',async()=>{
 const h=harness(),zone=h.$('joy-left');
 const pointer=id=>({pointerId:id,button:0,clientX:100,clientY:60,preventDefault(){}});
 zone.onpointerdown(pointer(1));await settle();h.failStatus(true);await h.tick(250);h.failStatus(false);await h.tick(250);
 const at=h.calls.length;zone.onpointermove({...pointer(1),clientY:20});await h.tick(50);
 assert.equal(h.calls.slice(at).some(nonzero),false);
 zone.onpointerdown(pointer(2));await settle();await settle();
 assert.deepEqual(h.calls.filter(nonzero).at(-1).body.twist,[.2,0,0]);
});

test('native late nonzero command ACK cannot unfreeze input or satisfy a newer neutral acknowledgement',async()=>{
 const h=harness();h.deferCommand();await h.key('w');
 h.failStatus(true);await h.tick(250);h.failStatus(false);await h.tick(250);await h.releaseCommand();
 assert.equal(h.snapshot().inputPaused,true);assert.equal(h.snapshot().neutralPending,true);
 const at=h.calls.length;await h.tick(50);assert.equal(h.calls.slice(at).some(nonzero),false);
 assert.deepEqual(h.calls.at(-1).body.twist,[0,0,0]);
});

test('native explicit stop still requests OFF and timer cannot restore the old movement',async()=>{
 for(const method of ['button','Space']){
  const h=harness();await h.key('w');if(method==='button')h.$('stop').click();else await h.key(' ');
  await settle();const at=h.calls.length;await h.tick(250);await h.tick(50);
  assert.equal(h.calls.filter(row=>row.url==='/api/mode').at(-1).body.mode,'off');
  assert.equal(h.calls.slice(at).some(nonzero),false);assert.equal(h.snapshot().inputPaused,true);
 }
});

test('native neutral acknowledgement precedes the first newly requested move after an outage',async()=>{
 const h=harness();await h.key('w');h.failCommand(true);await h.tick(50);h.failCommand(false);
 const at=h.calls.length;await h.keyup('w');await h.key('w');
 assert.deepEqual(h.calls.slice(at).filter(row=>row.url==='/api/command').map(row=>row.body.twist),[[0,0,0],[.2,0,0]]);
});
