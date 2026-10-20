/* Explicit fixed-program updates only. Loading and navigation never flash. */
(function(scope){
  'use strict';
  const PROGRAM_URL='/api/program-version';
  const validVersion=value=>typeof value==='string'&&value.trim().length>0&&value.length<=160;
  function fresh(detail,now){
    const s=detail?.status;
    return Boolean(!detail?.error&&!detail?.paused&&s&&Number.isInteger(s.mcu_seq)&&s.mcu_seq>=0&&s.mcu_seq<=0xffffffff&&
      Number.isFinite(s.received)&&s.received>0&&Number.isFinite(s.feedback_age_ms)&&s.feedback_age_ms>=0&&s.feedback_age_ms<150&&
      Number.isFinite(detail.checkedAt)&&now-detail.checkedAt>=0&&now-detail.checkedAt<1800);
  }
  function assessVersion(detail,program,now){
    const s=detail?.status,current=validVersion(s?.app_code_version)?s.app_code_version.trim():null;
    const latest=validVersion(program?.latest_version)?program.latest_version.trim():null;
    const packageReady=Boolean(latest&&program?.package_ready===true&&program.mcu_contract_revision===3);
    const contract=s?.mcu_contract_revision??s?.calibration?.mcu_contract_revision;
    const snapshot=s?.snapshot_version??s?.calibration?.snapshot_version;
    const isLatest=Boolean(latest&&current===latest&&fresh(detail,now)&&contract===3&&snapshot===3&&s.calibration_supported===true);
    return {current,latest,packageReady,isLatest};
  }
  function canUpdate(job,detail,calibration,now,program){
    if(program!==undefined){const version=assessVersion(detail,program,now);if(!version.packageReady||version.isLatest)return false;}
    if(job?.active===true||!fresh(detail,now)||detail.status.enabled!==false)return false;
    if(detail.status.mode==='read_only')return detail.readOnly===true&&detail.status.commissioning_motion_locked===true;
    return detail.status.mode==='wireless_calibration'&&detail.status.policy_locked===true&&
      calibration?.fresh===true&&calibration.policyLocked===true&&calibration.allOff===true&&
      calibration.interlocked!==true&&calibration.busy!==true&&calibration.s?.armed?.active===false;
  }
  function canUsb(job,detail,calibration,now,program){
    const version=assessVersion(detail,program,now);
    return version.packageReady&&!version.isLatest&&job?.active!==true&&calibration?.interlocked!==true&&
      calibration?.busy!==true&&calibration?.homeActive!==true&&calibration?.s?.armed?.active!==true;
  }
  function connected(detail,calibration,now){
    return fresh(detail,now)||(calibration?.connected===true&&calibration.fresh===true);
  }
  function updateReason(job,detail,calibration,now,program){
    const version=assessVersion(detail,program,now);
    if(job?.active===true)return '当前任务正在处理。';
    if(version.isLatest)return '当前程序已安装，无需重复烧录。';
    if(program?.install_method==='reviewed_delta'&&!version.isLatest)return program.reviewed_bundle_ready?'新版更新包已就绪，待上电后核验更新。':'新版更新包正在准备。';
    if(!version.packageReady)return '没有已验证的可用烧录包。';
    if(!fresh(detail,now))return '等待无线连接和新鲜反馈后可烧录。';
    if(!canUpdate(job,detail,calibration,now,program))return '等待当前操作结束并确认舵机全部卸力后可烧录。';
    return '';
  }
  if(typeof module==='object'&&module.exports){module.exports={PROGRAM_URL,fresh,assessVersion,canUpdate,canUsb,connected,updateReason};return;}
  const get=id=>document.getElementById(id);
  if(!get('comm-update'))return;
  let detail=null,program=null,job={active:false},posting=false,uncertain=false,requestStartedAt=null,priorJobStartedAt=null,lastError=null;
  let versionFetching=false,lastKnownVersion=null;
  const calibration=()=>scope.duckWirelessCalibration?.view();
  const shared=()=>scope.DuckLiveRuntime;
  const sharedReady=()=>typeof shared()?.setPosting==='function'&&typeof shared()?.acceptJob==='function';
  function render(){
    if(document.hidden)return;
    const version=assessVersion(detail,program,Date.now()),running=posting||job.active===true;
    const online=connected(detail,calibration(),Date.now()),reason=updateReason(job,detail,calibration(),Date.now(),program);
    const usbAllowed=sharedReady()&&!running&&!uncertain&&!document.hidden&&scope.DUCK_STATIC_SAMPLING!==true&&canUsb(job,detail,calibration(),Date.now(),program);
    get('comm-discover').disabled=online||!sharedReady()||running||uncertain||document.hidden||scope.DUCK_STATIC_SAMPLING===true;
    get('comm-discover').textContent=online?'无线已连接':'重新连接无线';
    get('comm-discover').title=online?'已自动连接，无需再次点击。':'未连上时重新搜索主控；页面会继续自动尝试连接。';
    get('comm-update').disabled=!sharedReady()||running||uncertain||document.hidden||scope.DUCK_STATIC_SAMPLING===true||!canUpdate(job,detail,calibration(),Date.now(),program);
    get('comm-update').textContent=version.isLatest?'无线烧录（无需）':'无线烧录';
    get('comm-update').title=reason||'安装可用的无线程序更新。';
    get('comm-usb').textContent='USB 首次烧录';
    get('comm-usb').disabled=!usbAllowed;
    const current=version.current||lastKnownVersion;
    get('comm-version').textContent=current?(fresh(detail,Date.now())?'主控 ':'上次 ')+current+(version.isLatest?' · 已是最新':version.latest&&current!==version.latest?'\n最新 '+version.latest:''):
      '主控版本 —'+(version.latest?'\n最新 '+version.latest:'');
    get('comm-version').dataset.latest=String(version.isLatest);
    const progress=get('comm-progress');progress.hidden=!running;
    progress.value=Number.isFinite(job.progress)?Math.max(0,Math.min(100,job.progress)):0;
    if(uncertain)get('comm-result').textContent='烧录结果待确认，正在读取任务状态。';
    else if(running)get('comm-result').textContent=posting?'正在提交…':(job.message||'正在处理…');
    else if(job.active!==true&&(job.stage==='not_needed'||['already_current','not_needed'].includes(job.outcome)))get('comm-result').textContent='已是最新，已跳过烧录。';
    else if(job.stage==='complete'&&['wireless_calibration_update','usb_calibration_install'].includes(job.mode))get('comm-result').textContent='程序已更新；日常用无线连接。';
    else if(['failed','incomplete','interrupted'].includes(job.stage)&&requestStartedAt)get('comm-result').textContent=job.message||'烧录未完成，请检查连接。';
    else if(lastError)get('comm-result').textContent=lastError;
    else if(version.isLatest)get('comm-result').textContent='无线已自动连接，无需再点连接；当前程序无需重复烧录。';
    else if(online)get('comm-result').textContent=reason||'无线已连接。有可用程序，可点无线烧录更新。';
    else get('comm-result').textContent='正在自动连接；未连上时可点重新连接无线。';
  }
  async function readVersion(){
    if(versionFetching||document.hidden)return;versionFetching=true;
    try{const response=await fetch(PROGRAM_URL,{cache:'no-store'});if(!response.ok)throw Error();const data=await response.json();
      program=validVersion(data.latest_version)&&data.mcu_contract_revision===3?data:null;
    }catch{program=null;}finally{versionFetching=false;render();}
  }
  async function submit(kind){
    if(!sharedReady()||posting||job.active===true||uncertain||document.hidden||scope.DUCK_STATIC_SAMPLING===true)return;
    const cal=calibration();
    if(kind==='discover'&&connected(detail,cal,Date.now()))return;
    if(kind==='wireless'&&!canUpdate(job,detail,cal,Date.now(),program))return;
    if(kind==='usb'&&!canUsb(job,detail,cal,Date.now(),program))return;
    const endpoint=kind==='usb'?'/api/wireless-calibration/usb':kind==='wireless'?'/api/wireless-calibration/update':'/api/provision/discover';
    const body=kind==='usb'?{acknowledged:true,use_saved_wifi:true,usb_connected:true,servo_power_off:true}:{acknowledged:true};
    posting=true;requestStartedAt=Date.now();priorJobStartedAt=job.started_at??null;lastError=null;
    shared()?.setPosting(true);render();
    const abort=new AbortController(),timer=setTimeout(()=>abort.abort(),6000);
    try{
      const reply=await fetch(endpoint,{method:'POST',headers:{'Content-Type':'application/json','X-Token':scope.COMMISSION_TOKEN},body:JSON.stringify(body),signal:abort.signal});
      const data=await reply.json();
      if(!reply.ok){lastError=typeof data.error==='string'?data.error:'流程未开始';requestStartedAt=null;return;}
      if(typeof data.active!=='boolean'){uncertain=true;return;}
      job=data;shared()?.acceptJob(data);
    }catch{uncertain=true;}
    finally{
      clearTimeout(timer);posting=false;shared()?.setPosting(uncertain);
      // Shared status reads can identify the original job; never retry this POST.
      if(typeof readProvision==='function')void readProvision();render();
    }
  }
  get('comm-usb').addEventListener('click',event=>{
    if(!event?.isTrusted)return;
    if(!sharedReady()||posting||job.active===true||uncertain||document.hidden||scope.DUCK_STATIC_SAMPLING===true||!canUsb(job,detail,calibration(),Date.now(),program))return;
    void submit('usb');
  });
  get('comm-discover').addEventListener('click',()=>submit('discover'));
  get('comm-update').addEventListener('click',()=>submit('wireless'));
  scope.addEventListener('duck:live-status',event=>{detail=event.detail;const value=detail?.status?.app_code_version;if(validVersion(value))lastKnownVersion=value.trim();render();});
  scope.addEventListener('duck:wireless-calibration',render);
  scope.addEventListener('duck:static-sampling',render);
  scope.addEventListener('duck:page',event=>{if(['imu','link'].includes(event.detail?.page))void readVersion();render();});
  scope.addEventListener('duck:provision-state',event=>{
    job=event.detail||{};const started=Date.parse(job.started_at);
    // An older idle poll cannot authorize retry after an unanswered mutation.
    if(uncertain&&requestStartedAt&&job.started_at!==priorJobStartedAt&&Number.isFinite(started)&&started>=requestStartedAt){uncertain=false;shared()?.setPosting(false);}
    if(job.stage==='complete'&&job.active!==true)void readVersion();render();
  });
  document.addEventListener('visibilitychange',()=>{if(!document.hidden)void readVersion();render();});
  setInterval(render,500);render();void readVersion();
})(typeof window==='undefined'?globalThis:window);
