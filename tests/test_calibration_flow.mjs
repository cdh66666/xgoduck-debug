/* Run the actual calibration adapter with an offline asynchronous backend. */
import assert from 'node:assert/strict';
import fs from 'node:fs';
import vm from 'node:vm';
import path from 'node:path';
import {fileURLToPath} from 'node:url';
const root=path.resolve(path.dirname(fileURLToPath(import.meta.url)),'../pc');
const IDS=[10,11,12,13,14,20,21,22,23,24,30,31,32,33,34],nodes=new Map(),intervals=[],calls=[],events=[];
const node=id=>{if(!nodes.has(id))nodes.set(id,{textContent:'',disabled:false,value:0,hidden:false,open:false,dataset:{},attrs:new Map(),append(){},showModal(){this.open=true;},close(){this.open=false;},setAttribute(k,v){this.attrs.set(k,v);},removeAttribute(k){this.attrs.delete(k);}});return nodes.get(id);};
const data={maintenance:{integrated:true,phase:'maintenance',active:true},calibration:{active:false,operation:'recalibrate',run_id:'old-run',held_mask:32767,saved:Object.fromEntries(IDS.map(id=>[id,{zero:2048}])),unknown:{},completed:IDS,prior_saved_count:15,current_saved_ids:[],current_batch_completed:null,message:'旧批次已完成',error:''}};
let run=0,rejectAction=false;
const window={dispatchEvent:e=>events.push(e)};
vm.runInNewContext(fs.readFileSync(path.join(root,'assets/bench-calibration.js'),'utf8'),{
 window,document:{hidden:false,getElementById:node,createElement:()=>({append(){}}),body:{append(){}}},
 fetch:async(url,options)=>{
  const body=options.body?JSON.parse(options.body):null;calls.push({url,body});
  if(body){
   if(rejectAction)return {ok:false,json:async()=>({detail:'主控无线未连接'})};
   if(body.action==='recalibrate')Object.assign(data.calibration,{active:true,operation:'recalibrate',run_id:'run-'+(++run),completed:[],current_saved_ids:[],current_batch_completed:[],message:'正在准备',error:''});
   if(body.action==='stop')data.calibration.message='已取消后续轴';
   return {ok:true,json:async()=>structuredClone(data.calibration)};
  }
  return {ok:true,json:async()=>structuredClone(data)};
 },
 AbortSignal,Date,Object,Number,String,Set,CustomEvent:class{constructor(type){this.type=type;}},setInterval:fn=>intervals.push(fn)
});
const settle=()=>new Promise(resolve=>setImmediate(resolve));await settle();
let checks=0;const check=(condition,message)=>{assert.ok(condition,message);checks++;},equal=(a,b,message)=>{assert.deepEqual(a,b,message);checks++;};
const posts=()=>calls.filter(call=>call.body),refresh=async()=>{intervals[0]();await settle();};
check(posts().length===0,'page load is GET only');
window.DuckCalibration.openFlow();
equal(node('calibration-flow-dialog').dataset.phase,'placement','old completed run never appears as a new success');
check(node('calibration-flow-preview').attrs.get('src')==='/assets/calibration-tray.html?purpose=calibration','actual R7 placement preview loads on open');
check(!node('calibration-all').disabled,'old held mask cannot lock a new recalibration');
check(posts().length===0,'opening preview never enters maintenance or calibrates');
await node('calibration-all').onclick();
equal(posts().map(call=>call.body.action),['recalibrate'],'one confirmation sends only the new combined action');
check(posts()[0].body.confirmed===true,'placement confirmation is explicit');
equal(node('calibration-flow-dialog').dataset.phase,'running','accepted run shows progress');
check(node('calibration-panel-close').disabled&&window.DuckCalibration.isBusy(),'close blocked during actual active run');
equal(node('calibration-progress').value,0,'old fifteen do not fill the new progress');
data.calibration.completed=[10,11,12];data.calibration.current_id=13;await refresh();
equal(node('calibration-progress').value,3,'progress follows current successful axes');
data.calibration.active=false;data.calibration.completed=IDS;data.calibration.current_saved_ids=IDS;data.calibration.current_batch_completed=IDS;data.calibration.message='校准成功，15颗新零位已保存。';await refresh();
equal(node('calibration-flow-dialog').dataset.phase,'success','matching complete run produces success');
check(!node('calibration-panel-close').disabled&&!node('calibration-done').hidden,'success offers close');
equal(posts().length,1,'completion never sends movement or unloads');
node('calibration-done').onclick();check(events.some(event=>event.type==='duck-calibration-close'),'done asks the parent UI to close');
window.DuckCalibration.closeFlow();check(!node('calibration-flow-preview').attrs.has('src'),'close unloads only preview');
equal(posts().length,1,'closing preview never writes the robot');
window.DuckCalibration.openFlow();
equal(node('calibration-flow-dialog').dataset.phase,'placement','reopen requires a fresh placement confirmation');
check(!node('calibration-all').disabled,'second full calibration remains available while previous axes hold');
await node('calibration-all').onclick();
equal(posts().filter(call=>call.body.action==='recalibrate').length,2,'new placement confirmation starts exactly one second run');
equal(node('calibration-progress').value,0,'second run progress starts fresh');
data.calibration.completed=[10];await window.DuckCalibration.stop();
check(node('calibration-panel-close').disabled,'stop request keeps progress visible until worker ends');
data.calibration.active=false;data.calibration.current_batch_completed=[10];await refresh();
equal(node('calibration-flow-dialog').dataset.phase,'failed','cancelled partial run never becomes a success');
check(!node('calibration-panel-close').disabled,'cancelled completed worker permits close');
rejectAction=true;await node('calibration-all').onclick();
check(/主控无线未连接/.test(node('calibration-entry-error').textContent)&&!node('calibration-entry-error').hidden,'failed confirmation is visible');
equal(posts().filter(call=>call.body.action==='recalibrate').length,3,'failed request is never replayed automatically');
check(!calls.some(call=>['/api/mode','/api/command','/api/pick','/api/roulade'].includes(call.url)),'calibration flow never starts motion models');
console.log(JSON.stringify({result:'PASS',checks,hardware_connected:false,scope:'single confirmation recalibrate flow, true run progress, repeat held calibration, cancellation and close without writes; actual adapter offline'}));
