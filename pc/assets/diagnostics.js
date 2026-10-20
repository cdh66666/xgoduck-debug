/* Read-only diagnostic snapshots. Never retries an action or sends device targets. */
(function(scope){
  'use strict';
  const pick=(value,keys)=>Object.fromEntries(keys.filter(k=>value?.[k]!==undefined).map(k=>[k,value[k]]));
  function age(status,checkedAt,now){
    const sourceAge=status?.age_ms??status?.feedback_age_ms;
    return Number.isFinite(sourceAge)&&sourceAge>=0&&Number.isFinite(checkedAt)&&checkedAt>0&&Number.isFinite(now)&&now>=checkedAt?
      sourceAge+now-checkedAt:Infinity;
  }
  function summarize(packet,checkedAt,now=Date.now()){
    const s=packet?.status,c=packet?.connection,ms=age(s,checkedAt,now);
    const online=c?.online===true,fresh=online&&packet?.fresh!==false&&s?.fresh!==false&&ms>=0&&ms<150;
    const allOff=fresh&&s?.all_torque_off_verified===true&&s?.session_active===false&&
      s?.armed?.active!==true&&s?.home?.active!==true;
    return {online,fresh,age_ms:Number.isFinite(ms)?Math.round(ms):null,
      connection:!online?'未连接':fresh?'反馈新鲜':'反馈过期',
      torque:allOff?'已确认卸力':fresh&&(s?.session_active===true||s?.armed?.active===true||s?.home?.active===true)?'控制会话中':'卸力待确认',
      discovery:c?.rediscovery?.active===true?'正在自动搜索主控':c?.rediscovery?.state==='waiting'?
        '自动搜索等待重试':'',
      status:pick(s,['boot_id','version','sequence','mcu_seq','mcu_state','last_received_req_seq','age_ms','feedback_age_ms','all_torque_off_verified','session_active']),
      home:pick(s?.home,['phase','error','active','release_verified'])};
  }
  function eventSummary(e){
    return pick(e,['at_unix_s','op','duration_ms','http_status','ok','reason','reported_fault','home_error','home_phase','home_stage','release_verified','all_torque_off_verified']);
  }
  function eventText(e){
    const failure=e?.ok!==true||e?.http_status>=400;
    const fault=e?.reported_fault??e?.home_error;
    return [e?.op||'未知操作',failure?'未成功':'请求成功',
      Number.isFinite(e?.duration_ms)?Math.round(e.duration_ms)+' ms':null,
      e?.http_status?'HTTP '+e.http_status:null,e?.reason||null,
      fault!==undefined&&fault!==null&&fault!==0?'错误 '+fault:null,e?.home_stage||null].filter(Boolean).join(' · ');
  }
  if(typeof module==='object'&&module.exports){module.exports={summarize,eventSummary,eventText};return;}
  const get=id=>document.getElementById(id),dialog=get('diagnostics-dialog');
  if(!dialog)return;
  let packet=null,checkedAt=0,service=null,serviceAt=0,healthReading=false,events=[],model=null,reading=false,request=null,lastRead=0,failures=[];
  const set=(id,value)=>{const n=get(id);if(n&&n.textContent!==value)n.textContent=value;};
  function render(){
    const v=summarize(packet,checkedAt),serviceAge=Date.now()-serviceAt;
    set('diag-service',!serviceAt?'待读取':serviceAge>=15000?'需要刷新':service?.ok===true?'运行中':'服务不可达');
    set('diag-controller',v.connection+(v.age_ms!==null?' · '+v.age_ms+' ms':''));
    set('diag-torque',v.torque);get('diag-torque').dataset.state=v.torque==='已确认卸力'?'ok':'unknown';
    set('diag-identity',v.status.boot_id!==undefined?'Boot '+v.status.boot_id+' · 帧 '+(v.status.sequence??v.status.mcu_seq??'—'):'等待主控身份与帧号');
    set('diag-discovery',v.discovery||(!v.online?'请检查主控供电与同一 Wi-Fi；后台会自动重连。':''));
    set('diag-refresh-note',reading?'读取诊断快照…':lastRead?'快照 '+new Date(lastRead).toLocaleTimeString('zh-CN',{timeZone:'Asia/Shanghai',hour12:false})+
      (failures.length?' · 未读到：'+failures.join('、'):''):'尚未读取');
    get('diag-refresh').disabled=reading;get('diag-export').disabled=!lastRead||reading;
  }
  function renderEvents(){
    const list=get('diag-events');list.replaceChildren();
    for(const e of events.slice(-8).reverse()){
      const li=document.createElement('li');li.textContent=eventText(e);li.dataset.state=e.ok===true&&e.http_status<400?'ok':'fail';list.append(li);
    }
    if(!events.length){const li=document.createElement('li');li.textContent='本次电脑服务尚无操作记录；重启前记录不在此列表。';list.append(li);}
    const runtime=model?.runtime;
    const label={unknown:'状态待确认',offline:'主控离线',idle:'未运行',running:'运行中',fault:'故障',error:'读取失败',stopped:'已停止',stop_unconfirmed:'停止待确认'};
    set('diag-model',runtime?'模型：'+(label[runtime.state]||runtime.state||'状态未知')+(runtime.error?' · '+String(runtime.error):''):'模型状态未读到');
  }
  async function read(path,signal){
    const r=await fetch(path,{cache:'no-store',signal});if(!r.ok)throw Error('HTTP '+r.status);return r.json();
  }
  async function refresh(){
    if(reading||!dialog.open||document.hidden)return;
    reading=true;render();const abort=new AbortController();request=abort;
    const timer=setTimeout(()=>abort.abort(),6000);
    try{
      const names=['电脑服务','操作记录','主控反馈','模型状态'];
      const results=await Promise.allSettled(['/api/health','/api/wireless-calibration/actions','/api/wireless-calibration','/api/models'].map(async path=>({data:await read(path,abort.signal),at:Date.now()})));
      if(request!==abort||!dialog.open)return;
      failures=results.flatMap((r,i)=>r.status==='rejected'?[names[i]]:[]);
      service=results[0].status==='fulfilled'?pick(results[0].value.data,['ok','startup_read_only']):null;serviceAt=Date.now();
      events=results[1].status==='fulfilled'?(results[1].value.data.events||[]).map(eventSummary):[];
      if(results[2].status==='fulfilled'&&results[2].value.at>=checkedAt){packet=results[2].value.data;checkedAt=results[2].value.at;}
      model=results[3].status==='fulfilled'?{runtime:pick(results[3].value.data.runtime,['state','error','sent_frames','accepted_frame'])}:null;
      lastRead=Date.now();renderEvents();
    }finally{clearTimeout(timer);if(request===abort)request=null;reading=false;render();}
  }
  async function health(){
    if(healthReading||reading||!dialog.open||document.hidden)return;
    healthReading=true;const abort=new AbortController(),timer=setTimeout(()=>abort.abort(),1500);
    try{service=pick(await read('/api/health',abort.signal),['ok','startup_read_only']);}
    catch{service=null;}
    finally{clearTimeout(timer);serviceAt=Date.now();healthReading=false;render();}
  }
  get('diagnostics-open').addEventListener('click',()=>{dialog.showModal();void refresh();});
  get('diagnostics-close').addEventListener('click',()=>dialog.close());
  dialog.addEventListener('close',()=>{request?.abort();get('diagnostics-open').focus();});
  get('diag-refresh').addEventListener('click',()=>void refresh());
  get('diag-export').addEventListener('click',()=>{
    const data={schema:'duck-diagnostics-v1',exported_at:new Date().toISOString(),snapshot_at:new Date(lastRead).toISOString(),
      scope:'Read-only observations and request outcomes; physical acceptance is not implied',service,service_checked_at:serviceAt?new Date(serviceAt).toISOString():null,
      controller:summarize(packet,checkedAt),events,model,unavailable:failures};
    const url=URL.createObjectURL(new Blob([JSON.stringify(data,null,2)],{type:'application/json'}));
    const a=document.createElement('a');a.href=url;a.download='duck-diagnostics-'+Date.now()+'.json';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);
    set('diag-export-note','诊断记录已生成');
  });
  scope.addEventListener('duck:wireless-calibration',e=>{
    // The existing poll owns live feedback. This view cannot authorize controls.
    packet={...packet,status:e.detail?.status,fresh:e.detail?.fresh,
      connection:{...packet?.connection,online:scope.duckWirelessCalibration?.view()?.connected===true}};
    checkedAt=Date.now();if(dialog.open)render();
  });
  setInterval(()=>{if(dialog.open&&!document.hidden)render();},500);
  setInterval(()=>void health(),5000);
})(typeof window==='undefined'?globalThis:window);
