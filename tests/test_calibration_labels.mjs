/* Exercise current-vs-history labels with read-only calibration responses. */
import assert from 'node:assert/strict';
import fs from 'node:fs';
import vm from 'node:vm';
import path from 'node:path';
import {fileURLToPath} from 'node:url';
const root=path.resolve(path.dirname(fileURLToPath(import.meta.url)),'../pc');
const nodes=new Map(),intervals=[],calls=[];let rejectAction=false;
const node=id=>{if(!nodes.has(id))nodes.set(id,{textContent:'',disabled:false,value:0,dataset:{},append(){},showModal(){},close(){}});return nodes.get(id);};
const saved=Object.fromEntries([10,11,12,13,14,20,21,22,23,24,30,31,32,33,34].map(id=>[id,{zero:2048}]));
const data={maintenance:{integrated:true,phase:'official',active:false},calibration:{active:false,held_mask:0,saved,unknown:{},message:'已保存',last_batch:{elapsed_ms:4750,completed:Object.keys(saved)},prior_saved_count:15,current_saved_ids:[],current_batch_completed:null}};
const window={dispatchEvent(){}};
vm.runInNewContext(fs.readFileSync(path.join(root,'assets/bench-calibration.js'),'utf8'),{
 window,document:{hidden:false,getElementById:node,createElement:()=>({append(){}}),body:{append(){}}},
 fetch:async(url,options)=>{calls.push({url,options});if(options.method&&rejectAction)return {ok:false,json:async()=>({detail:'主控无线未连接'})};return {ok:true,json:async()=>structuredClone(data)};},
 AbortSignal,Date,Object,Number,String,CustomEvent:class{},setInterval:fn=>intervals.push(fn)
});
const settle=()=>new Promise(resolve=>setImmediate(resolve));await settle();
let checks=0;const equal=(actual,expected)=>{assert.equal(actual,expected);checks++;};
equal(node('sequence-count').textContent,'本次成功 0 / 15');
equal(node('calibration-history').textContent,'上次已保存 15 / 15 · 本次尚未保存');
equal(node('calibration-progress-text').textContent,'本次全身校准尚未开始');
equal(node('calibration-progress').value,0);
data.maintenance.phase='maintenance';intervals[0]();await settle();
equal(node('sequence-count').textContent,'本次成功 0 / 15');
equal(node('calibration-progress-text').textContent,'本次全身校准尚未开始');
equal(node('calibration-progress').value,0);
data.maintenance.phase='entering';data.calibration.active=true;intervals[0]();await settle();
equal(node('sequence-count').textContent,'0 / 15');
data.calibration.active=false;data.calibration.current_saved_ids=[10,10,22];data.calibration.current_batch_completed=[10,22];intervals[0]();await settle();
equal(node('sequence-count').textContent,'本次成功 2 / 15');
equal(node('calibration-history').textContent,'上次已保存 15 / 15 · 本次新成功 2 颗');
equal(node('calibration-progress-text').textContent,'2 / 15');
data.calibration.current_saved_ids=[];data.calibration.current_batch_completed=null;data.maintenance.phase='official';data.calibration.operation='enter';data.calibration.message='主控请求超时';intervals[0]();await settle();
equal(node('calibration-entry-error').hidden,false);
assert.match(node('calibration-entry-error').textContent,/进入校准未确认.*超时/);checks++;
assert.ok(calls.every(call=>!call.options.method),'status and label checks never send device writes');checks++;
rejectAction=true;await window.DuckCalibration.action('enter',true);
equal(node('calibration-entry-error').hidden,false);
assert.match(node('calibration-entry-error').textContent,/主控无线未连接.*不会自动重试/);checks++;
equal(calls.filter(call=>call.options.method==='POST').length,1);
equal(JSON.parse(calls.at(-1).options.body).action,'enter');
equal(node('sequence-count').textContent,'本次成功 0 / 15');
console.log(JSON.stringify({result:'PASS',checks,hardware_connected:false,scope:'historical receipts never become current by entering; actual-success provenance; entry error visible next to button; offline API only'}));
