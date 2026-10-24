/* No browser or robot: execute the real replay UI with local captured fixtures. */
import assert from 'node:assert/strict';
import fs from 'node:fs';
import vm from 'node:vm';
const source=fs.readFileSync(new URL('../pc/assets/motion-history.js',import.meta.url),'utf8');
const nodes=new Map(),requests=[],poses=[],modes=[],notes=[],callbacks=[];
class Element{
  constructor(){this.value='';this.textContent='';this.children=[];this.hidden=false;this.dataset={};this.width=960;this.height=58;}
  append(...children){this.children.push(...children);}
  replaceChildren(){this.children=[];}
  getContext(){return null;}
}
const $=id=>{if(!nodes.has(id))nodes.set(id,new Element());return nodes.get(id);};
const frames=[0,.04,.08,.5,.54].map((t,index)=>({t,status:{mode:'policy',enabled:true,
  gravity:index<3?[0,0,-1]:[1,0,0],angles:Array(15).fill(index),targets:Array(15).fill(index+1),
  command:[0,0,0],recovery:{phase:'walk'},mcu_seq:index}}));
const summary={id:'real-test',source:'robot-status',label:'真实记录',frames:5,start_unix_s:1000,duration_s:.54,archived:true,tilt_after_upright_at_s:.5};
let live={mode:'off',enabled:false},now=0;
const context={window:{},document:{body:{dataset:{}},hidden:false,getElementById:$,createElement:()=>new Element(),addEventListener(){}},
  location:{search:''},Date,URLSearchParams,performance:{now:()=>now},AbortSignal,
  setInterval(){},requestAnimationFrame:fn=>{callbacks.push(fn);return callbacks.length;},cancelAnimationFrame(){},
  fetch:async(url,options={})=>{requests.push({url,options});return {ok:true,json:async()=>url==='/api/bench/motions'?{items:[summary],recorder:{sampler_alive:true,writer_alive:true},last_sync_unix_s:Date.now()/1000}:{summary,frames}};}};
vm.runInNewContext(source,context);
const {Timeline,tilt,estimatedTilt,phaseLabel,create}=context.window.DuckMotionHistory;
const legacy={gravity:null,body_attitude_estimate:{basis:'raw_acceleration_only',gravity:[1,0,0]}};
assert.equal(tilt(legacy),null,'raw acceleration is not recorded filtered body attitude');
assert.equal(estimatedTilt(legacy),90,'optional estimate stays a separately typed display layer');
assert.equal(phaseLabel({mode:'off',enabled:false,recovery:{phase:'walk'}}),'卸力','OFF wins over a retained recovery phase');
assert.equal(phaseLabel({mode:'policy',enabled:true,recovery:{phase:'getup'}}),'起身模型');
assert.equal(phaseLabel({mode:'legacy',enabled:false}),'已卸力 · 旧原始反馈');
const timeline=new Timeline(frames);
assert.equal(timeline.at(.03).index,0,'seeking holds an actual recorded endpoint');
assert.equal(timeline.at(.06).row.status.angles[0],1,'no invented interpolation or model inference');
assert.equal(timeline.at(.4).gap,true,'source gap remains explicitly visible');
assert.equal(timeline.at(10).index,4,'end of capture never extrapolates');
assert.throws(()=>new Timeline([frames[1],frames[0]]),/时间轴/);
assert.equal(tilt(frames[3].status),90);
$('motion-speed').value='0.25';
const ui=create({getLive:()=>live,onMode:mode=>modes.push(mode),postPose:pose=>poses.push(pose),note:message=>notes.push(message)});
await new Promise(resolve=>setImmediate(resolve));
assert.equal(ui.active,false);
await ui.select('real-test');
assert.equal(ui.active,true);assert.equal(modes.at(-1),true);
assert.equal(poses.at(-1).type,'motion-replay-frame');assert.equal(poses.at(-1).index,0);
$('motion-next').onclick();assert.equal(poses.at(-1).index,1);
$('motion-seek').value='.4';$('motion-seek').oninput();assert.equal(poses.at(-1).gap,true);
$('motion-play').onclick();now=200;callbacks.at(-1)(now);
assert.equal(poses.at(-1).time,.08,'quarter-speed playback does not invent missing frames');
ui.live();assert.equal(ui.active,false);assert.equal(poses.at(-1).type,'motion-replay-end');assert.equal(modes.at(-1),false);
live={mode:'policy',enabled:true};ui.update();assert.equal($('motion-history').disabled,true);
const count=requests.length;await ui.select('real-test');assert.equal(requests.length,count,'cannot obscure active control with replay');
assert.match(notes.at(-1),/停止/);assert.equal(ui.active,false);
assert.ok(requests.every(row=>!row.options.method||row.options.method==='GET'),'selection/play/pause/seeking/return use GET only');
assert.ok(requests.every(row=>row.url.startsWith('/api/bench/motions')),'no hardware or model control endpoints');
console.log(JSON.stringify({result:'PASS',requests:requests.length,hardwareWrites:0,scope:'actual replay UI, recorded endpoints/gaps, phase labels and separate estimates'}));
