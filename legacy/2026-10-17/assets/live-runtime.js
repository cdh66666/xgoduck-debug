/* Read-only live/provision state. No legacy tutorial UI or automatic actions. */
let provisionState={active:false},provisionPosting=false;
function safeProvisionUrl(value){
  if(typeof value!=='string'||value.length>256)return null;
  try{const u=new URL(value),p=u.hostname.split('.').map(Number);
    if(u.protocol!=='http:'||u.username||u.password||u.search||u.hash||!['','/'].includes(u.pathname)||(u.port&&u.port!=='9527'))return null;
    return /^\d+\.\d+\.\d+\.\d+$/.test(u.hostname)&&p.every(n=>Number.isInteger(n)&&n>=0&&n<=255)&&
      (p[0]===10||p[0]===192&&p[1]===168||p[0]===172&&p[1]>=16&&p[1]<=31)?u.origin:null;
  }catch{return null;}
}
let liveUrl=null,liveStream=null,liveGeneration=0,liveAbort=null,liveFetching=false,jobFetching=false;
let liveLastAt=0,liveRetryAt=0,liveFailures=0,liveStreamSupported=false;
let imuEnabled=false,imuStream=null,imuGeneration=0,imuRetryAt=0,imuLastAt=0,imuStaleTimer=null;
let imuPage=document.querySelector?.('[data-page].active')?.dataset?.page||'imu';
window.DuckIMUDatagramEnabled=false;
const liveEmit=detail=>window.dispatchEvent(new CustomEvent('duck:live-status',{detail}));
const liveBusy=()=>document.hidden||provisionState.active===true||provisionPosting||window.DUCK_STATIC_SAMPLING===true;
const calibrationBusy=()=>{
  const view=window.duckWirelessCalibration?.view?.();
  return !!(view?.busy||view?.homeActive||view?.homeOwned||view?.s?.operation||view?.s?.session_active||view?.s?.armed?.active);
};
function cancelProvisionLive(){liveGeneration++;liveAbort?.abort();liveAbort=null;liveFetching=false;liveStream?.close();liveStream=null;liveLastAt=0;}
function boardSummaryDetail(detail){
  // Calibration needs connection/power indicators, not another pose stream.
  // Keep only existing observed status fields; no derived IMU/encoder frame.
  const status={},input=detail.status;
  for(const key of ['mcu_seq','received','feedback_age_ms','app_instance_id','mode','host_read_only','no_rpc_writes',
    'servo_ids','imu_ok','battery','servo_power','enabled','policy_locked','calibration_supported','calibration_fresh',
    'calibration_age_ms','all_torque_off_verified','torque_may_be_on','session_active','mcu_state','commissioning_motion_locked']){
    if(Object.hasOwn(input,key))status[key]=input[key];
  }
  return {...detail,status,summaryOnly:true};
}
function liveSseDetail(packet,receivedAt=Date.now()){
  const status=packet?.status,checkedAt=packet?.checked_at;
  if(packet?.error||!status||typeof status!=='object'||!Number.isFinite(status.feedback_age_ms)||status.feedback_age_ms<0||
    !Number.isFinite(checkedAt)||checkedAt<=0||!Number.isFinite(receivedAt))return {error:true};
  const transit=receivedAt-checkedAt*1000;
  // PC and browser share the wall clock; Date.now() truncates sub-milliseconds.
  if(!Number.isFinite(transit)||transit< -1)return {error:true};
  const age=status.feedback_age_ms+Math.max(0,transit);
  if(!Number.isFinite(age)||age>=150)return {error:true};
  return {status:{...status,feedback_age_ms:age},readOnly:true,checkedAt:receivedAt};
}
function syncLive(){
  syncImuStream();
  if(!liveUrl||liveBusy()){cancelProvisionLive();liveEmit({paused:true});return;}
  if(imuPage==='servo'){
    if(liveStream)cancelProvisionLive();
    if(calibrationBusy()){
      if(liveAbort)cancelProvisionLive();
      return;
    }
    if(Date.now()>=liveRetryAt)readProvisionLive();
    return;
  }
  if(typeof EventSource==='function'&&!liveStream){
    const generation=liveGeneration,source=new EventSource('/api/live?url='+encodeURIComponent(liveUrl));liveStream=source;
    const current=()=>generation===liveGeneration&&liveStream===source&&!liveBusy()&&imuPage!=='servo';
    source.addEventListener('status',event=>{if(!current())return;try{const now=Date.now(),detail=liveSseDetail(JSON.parse(event.data),now);if(!detail.error){liveLastAt=now;liveFailures=0;}liveEmit(detail);}catch{liveEmit({error:true});}});
    source.addEventListener('offline',()=>{if(current())liveEmit({error:true});});
    source.onerror=()=>{if(current()){source.close();liveStream=null;liveStreamSupported=false;liveRetryAt=Date.now()+1000;liveEmit({error:true});}};
    source.onopen=()=>{if(current()){liveStreamSupported=true;liveAbort?.abort();}};
  }else if(typeof EventSource!=='function')readProvisionLive();
}
async function readProvisionLive(){
  if(!liveUrl||liveBusy()||liveFetching||liveStreamSupported&&liveStream)return;
  const summaryOnly=imuPage==='servo';
  if(summaryOnly&&(calibrationBusy()||Date.now()<liveRetryAt))return;
  if(summaryOnly)liveRetryAt=Date.now()+1000;
  const generation=liveGeneration,url=liveUrl,abort=new AbortController();liveAbort=abort;liveFetching=true;
  const timer=setTimeout(()=>abort.abort(),3500),current=()=>generation===liveGeneration&&url===liveUrl&&!liveBusy()&&
    (imuPage==='servo')===summaryOnly&&(!summaryOnly||!calibrationBusy())&&!(liveStreamSupported&&liveStream);
  try{
    const r=await fetch('/api/board-status?url='+encodeURIComponent(url),{cache:'no-store',signal:abort.signal});
    const d=await r.json();if(!r.ok)throw Error();
    if(current()){
      const now=Date.now(),detail=liveSseDetail(d,now);
      if(detail.error)throw Error('Stale or invalid telemetry');
      liveLastAt=now;liveFailures=0;liveEmit(summaryOnly?boardSummaryDetail(detail):detail);
    }
  }
  catch{if(current()){liveFailures++;liveRetryAt=Date.now()+[summaryOnly?1000:500,1000,3000][Math.min(liveFailures-1,2)];liveEmit({error:true,summaryOnly});}}
  finally{clearTimeout(timer);if(liveAbort===abort){liveAbort=null;liveFetching=false;}}
}
function acceptProvision(d){
  if(!d||typeof d.active!=='boolean')return;
  provisionState=d;
  const url=safeProvisionUrl(d.configured_network_url)||safeProvisionUrl(d.network_url)||safeProvisionUrl(d.last_known_network_url);
  if(url&&url!==liveUrl){cancelProvisionLive();liveUrl=url;liveRetryAt=0;}
  window.dispatchEvent(new CustomEvent('duck:provision-state',{detail:{...d,active:d.active||provisionPosting,request_pending:provisionPosting}}));
  syncLive();
}
async function readProvision(){
  if(jobFetching)return;jobFetching=true;const abort=new AbortController(),timer=setTimeout(()=>abort.abort(),3500);
  try{const r=await fetch('/api/provision/status',{cache:'no-store',signal:abort.signal});if(r.ok)acceptProvision(await r.json());}
  catch{if(!liveUrl)liveEmit({error:true});}finally{clearTimeout(timer);jobFetching=false;}
}
window.DuckLiveRuntime={url:()=>liveUrl,acceptJob:acceptProvision,setPosting(value){
  if(provisionPosting===!!value)return;provisionPosting=!!value;
  window.dispatchEvent(new CustomEvent('duck:provision-state',{detail:{...provisionState,active:provisionState.active||provisionPosting,request_pending:provisionPosting}}));syncLive();
}};
document.addEventListener('visibilitychange',()=>{syncLive();if(!document.hidden)readProvision();});
window.addEventListener('duck:static-sampling',syncLive);
setInterval(readProvision,2000);
setInterval(()=>{
  if(imuPage==='servo'&&calibrationBusy()){if(liveAbort||liveStream)cancelProvisionLive();}
  else if(!liveBusy()&&Date.now()>=liveRetryAt){if(!liveStream)syncLive();if(imuPage!=='servo'&&!liveStreamSupported)readProvisionLive();}
  if(liveLastAt&&Date.now()-liveLastAt>1800){liveLastAt=0;liveEmit({error:true});}
},100);
readProvision();

// The authenticated UDP path carries only IMU observations. Board connection,
// battery and every control/lease status continue on their existing channels.
function imuEmit(detail){window.dispatchEvent(new CustomEvent('duck:imu-status',{detail}));}
function clearImuDeadline(){if(imuStaleTimer!==null)clearTimeout(imuStaleTimer);imuStaleTimer=null;imuLastAt=0;}
function closeImuStream(){imuGeneration++;imuStream?.close();imuStream=null;clearImuDeadline();}
function syncImuStream(){
  if(!imuEnabled)return;
  if(liveBusy()||imuPage!=='imu'){
    if(imuStream||imuStaleTimer!==null||imuLastAt){closeImuStream();imuEmit({paused:true});}
    return;
  }
  if(imuStream||Date.now()<imuRetryAt)return;
  if(typeof EventSource!=='function'){imuEmit({error:true});return;}
  const generation=imuGeneration,stream=new EventSource('/api/imu-live');imuStream=stream;
  const current=()=>generation===imuGeneration&&imuStream===stream&&!liveBusy()&&imuPage==='imu';
  stream.addEventListener('status',event=>{
    if(!current())return;
    try{
      const now=Date.now(),detail=liveSseDetail(JSON.parse(event.data),now);clearImuDeadline();
      if(!detail.error){
        imuLastAt=now;
        imuStaleTimer=setTimeout(()=>{if(current()){clearImuDeadline();imuEmit({error:true});}},150-detail.status.feedback_age_ms);
      }
      imuEmit(detail);
    }
    catch{clearImuDeadline();imuEmit({error:true});}
  });
  stream.addEventListener('offline',()=>{if(current()){clearImuDeadline();imuEmit({error:true});}});
  stream.onerror=()=>{if(current()){closeImuStream();imuRetryAt=Date.now()+1000;imuEmit({error:true});}};
}
async function readImuConfig(){
  try{
    const response=await fetch('/api/imu-transport',{cache:'no-store'});
    if(!response.ok)return;
    const config=await response.json();imuEnabled=config.enabled===true;
    window.DuckIMUDatagramEnabled=imuEnabled;
    window.dispatchEvent(new CustomEvent('duck:imu-mode',{detail:{enabled:imuEnabled}}));
    syncImuStream();
  }catch{ /* Existing TCP behavior remains when this opt-in API is absent. */ }
}
window.addEventListener('duck:page',event=>{
  const page=event.detail?.page||imuPage;
  if(page!==imuPage){cancelProvisionLive();closeImuStream();liveRetryAt=0;imuRetryAt=0;imuPage=page;}
  syncLive();
});
setInterval(syncImuStream,100);
readImuConfig();
