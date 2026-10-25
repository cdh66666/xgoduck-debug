/* Explicit manufacturer-maintenance controls. All page-load requests are GET. */
(() => {
  'use strict';
  const $=id=>document.getElementById(id),IDS=[10,11,12,13,14,20,21,22,23,24,30,31,32,33,34];
  let data=null,selected=10,busy=false,posting=false,pending=null,errorText='';
  const text=(id,value)=>{if($(id).textContent!==value)$(id).textContent=value;};
  const maintenance=()=>!!data?.maintenance?.active||!!data?.calibration?.active;
  const isBusy=()=>posting||!!data?.calibration?.active;
  let flowRequested=false,flowRunId=null,openedRunId=null;
  function openFlow(){
    flowRequested=false;flowRunId=null;openedRunId=data?.calibration?.run_id??null;errorText='';
    $('calibration-flow-preview').setAttribute('src','/assets/calibration-tray.html?purpose=calibration');
    $('calibration-single-details').open=false;$('calibration-details').open=false;paint();
  }
  function closeFlow(){$('calibration-flow-preview').removeAttribute('src');}
  function paintFlow(){
    const c=data?.calibration,active=isBusy(),ids=Array.isArray(c?.completed)?c.completed:[];
    if(flowRequested&&!flowRunId&&c?.run_id&&c.run_id!==openedRunId)flowRunId=c.run_id;
    const ownsRun=flowRequested&&!!flowRunId&&c?.run_id===flowRunId;
    const failed=!!(errorText||c?.error||Object.keys(c?.unknown||{}).length||/取消|失败|未确认|超时/.test(c?.message||''));
    const success=ownsRun&&!active&&c?.operation==='recalibrate'&&ids.length===15&&!failed;
    const phase=active?'running':success?'success':flowRequested?'failed':'placement';
    $('calibration-flow-dialog').dataset.phase=phase;
    $('calibration-panel-close').disabled=active;
    $('calibration-all').disabled=!data||active;
    $('calibration-all').hidden=success||active;
    text('calibration-all',phase==='failed'?'已重新摆好，再次校准':'已摆好，开始校准');
    $('calibration-done').hidden=!success;
    $('calibration-flow-progress').hidden=phase==='placement';
    $('calibration-stop').hidden=!active;
    $('sequence-count').hidden=phase==='placement';
    if(active){text('calibration-mode','正在校准，请保持托盘和鸭子不动。');text('sequence-count',`${ids.length} / 15`);}
    else if(success){text('calibration-mode','校准成功 · 15颗中位已保存'+(c?.calibration_loads_motor===false?' · 保持卸力':''));text('sequence-count','15 / 15');}
    else if(flowRequested){text('calibration-mode','本次校准未完成，请查看提示。');text('sequence-count',`${ownsRun?ids.length:0} / 15`);}
    else{text('calibration-mode','把鸭子放入 R7 托盘，确认全身贴合定位槽。');text('sequence-result',c?.calibration_loads_motor===false?'确认后逐颗保存机械中位，全程卸力，不执行回正或站立。':'确认后将自动卸力、进入校准，并逐颗保存中位；托盘内须使用支持卸力校准的固件。');}
    if(active||ownsRun){$('calibration-progress').value=ids.length;text('calibration-progress-text',`${ids.length} / 15${active&&c?.current_id?' · 当前 ID '+c.current_id:success?' · 已完成':''}`);}
  }
  const dialog=document.createElement('dialog');dialog.id='calibration-confirm-dialog';
  dialog.innerHTML='<h2>校准操作预告</h2><p id="calibration-confirm-text"></p><div class="bench-actions"><button id="calibration-cancel" class="quiet">取消</button><button id="calibration-confirm">确认本次操作</button></div>';
  document.body.append(dialog);
  const explanations={
    enter:'将进入校准维护，请先全部卸力并支撑机身。进入后不会自动保存中位。',
    restore:'将核对全部卸力与本次零位保存结果，再恢复正式模型固件和程序。恢复后保持 OFF，动作须另行启动。',
    all:'确认鸭子全身已贴合托盘定位槽，每个关节均处于机械中位且全部卸力。将依次对15轴保存新中位。停止会取消后续轴。',
    calibrate:()=>`将把 ID ${selected} 的当前手摆姿态保存为新的机械中位。${data?.calibration?.calibration_loads_motor===false?'全程保持卸力。':'托盘内须使用支持卸力校准的固件，扭矩状态以主控读回为准。'}`
  };
  async function get(){
    if(busy)return;busy=true;
    try{const r=await fetch('/api/bench/calibration',{cache:'no-store',signal:AbortSignal.timeout(3500)});if(!r.ok)throw Error('校准状态读取失败');data=await r.json();paint();}
    catch(e){text('calibration-mode',e.message);text('calibration-entry-error','校准状态未确认：'+e.message);$('calibration-entry-error').hidden=false;}finally{busy=false;}
  }
  async function action(action,confirmed=false){
    if(posting&&action!=='stop')return;posting=true;errorText='';
    if(action==='recalibrate'){flowRequested=true;flowRunId=null;openedRunId=data?.calibration?.run_id??null;}paintFlow();
    try{
      const r=await fetch('/api/bench/calibration/action',{method:'POST',headers:{'Content-Type':'application/json','X-Token':window.COMMISSION_TOKEN},body:JSON.stringify({action,id:selected,confirmed}),signal:AbortSignal.timeout(5000)});
      const value=await r.json();if(!r.ok)throw Error(value.detail||'操作未确认');
      if(action==='recalibrate')flowRunId=value.run_id??null;
      data={maintenance:data?.maintenance||{},calibration:value};text('sequence-result',value.message);await get();
    }catch(e){errorText=e.message+'；不会自动重试。';text('sequence-result',errorText);}finally{posting=false;paint();}
  }
  function prepare(op){
    if(data?.maintenance?.integrated&&['enter','restore'].includes(op)){action(op,true);return;}
    pending=op;const description=explanations[op];text('calibration-confirm-text',typeof description==='function'?description():description);dialog.showModal();
  }
  $('calibration-cancel').onclick=()=>dialog.close();
  $('calibration-confirm').onclick=()=>{const action=pending;dialog.close();action&&window.DuckCalibration.action(action,true);};
  for(const [id,op] of [['calibration-enter','enter'],['calibration-restore','restore'],['sequence-capture','calibrate']])$(id).onclick=()=>prepare(op);
  $('calibration-all').onclick=()=>action('recalibrate',true);
  $('calibration-done').onclick=()=>window.dispatchEvent(new CustomEvent('duck-calibration-close'));
  for(const [id,op] of [['calibration-release','release'],['calibration-read','read'],['calibration-stop','stop']])$(id).onclick=()=>action(op);
  $('calibration-next').onclick=()=>{selected=IDS[(IDS.indexOf(selected)+1)%IDS.length];window.dispatchEvent(new CustomEvent('duck-calibration-select',{detail:selected}));if(data?.maintenance?.phase==='maintenance')action('read');};
  function paint(){
    if(!data)return;
    const m=data.maintenance,c=data.calibration,ready=m.phase==='maintenance',active=c.active||posting,unknown=Object.keys(c.unknown||{}).length;
    if(m.integrated){
      text('calibration-enter','进入校准');text('calibration-restore','退出校准');
      text('calibration-mode',ready?'校准已就绪':'先进入校准，再保存中位');
    }
    else text('calibration-mode',({idle:'正式模型模式 · 进入维护后才能写校准',official:'正式模型模式 · 进入维护后才能写校准',maintenance:'厂家校准维护 · 模型控制已暂停',entering:'正在切换厂家维护固件',restoring:'正在恢复正式模型程序',unknown:'维护切换结果未确认，需核对记录',restored:'正式模型已恢复 · OFF'}[m.phase]||m.phase||'状态待读取')+(m.error?' · '+m.error:''));
    const entryError=errorText||c.error||m.error||(!ready&&!active&&c.operation==='enter'?'进入校准未确认：'+(c.message||'请检查主控读回'):'');
    text('calibration-entry-error',entryError);$('calibration-entry-error').hidden=!entryError;
    $('calibration-enter').disabled=active||m.active;
    $('calibration-restore').disabled=active||!ready||unknown>0||c.held_mask!==0;
    $('sequence-capture').disabled=active||!ready||unknown>0||!!(c.held_mask&(1<<IDS.indexOf(selected)));
    $('calibration-release').disabled=$('calibration-read').disabled=active||!ready;
    $('calibration-stop').disabled=!m.active&&!c.active;
    $('calibration-next').disabled=active;
    const currentSaved=Array.isArray(c.current_saved_ids)?new Set(c.current_saved_ids.map(Number).filter(id=>IDS.includes(id))):new Set();
    const priorCount=Number.isInteger(c.prior_saved_count)?c.prior_saved_count:Object.keys(c.saved||{}).length;
    const freshBatch=Array.isArray(c.current_batch_completed)?c.current_batch_completed:null;
    const batch=c.active&&c.operation==='all'?c.completed:(freshBatch??[]);
    $('calibration-progress').value=batch?.length||0;
    text('calibration-progress-text',batch?.length?`${batch.length} / 15${c.active&&c.operation==='all'?' · 当前 ID '+c.current_id:''}`:'本次全身校准尚未开始');
    text('sequence-count',`本次成功 ${currentSaved.size} / 15`);
    text('calibration-history',`上次已保存 ${priorCount} / 15${currentSaved.size?' · 本次新成功 '+currentSaved.size+' 颗':' · 本次尚未保存'}`);
    text('sequence-result',errorText||c.message+(unknown?' · 未确认：'+Object.keys(c.unknown).map(id=>'ID '+id).join('、'):''));
    text('calibration-read-status',m.integrated&&c.last_read_at?'上次寄存器读回 '+new Date(c.last_read_at*1000).toLocaleTimeString():'尚无本次寄存器读回');
    const timing=c.timing_ms||{};
    text('calibration-timing',(c.last_batch?'全身 '+(c.last_batch.elapsed_ms/1000).toFixed(2)+' 秒 · ':'')+(Number.isFinite(timing.total_ms)?`上次操作 ${(timing.total_ms/1000).toFixed(2)} 秒${m.integrated?'':' · '+(timing.reused?'复用连接':'首次连接')}${Number.isFinite(timing.calibrate_ms)?' · 校准读回 '+(timing.calibrate_ms/1000).toFixed(2)+' 秒':''}`:'耗时将在本次操作后显示'));
    const r=c.last_read;if(ready&&r?.id===selected&&(!m.integrated||[1,2,3].includes(r.op))){text('sequence-selected',`ID ${selected} · ${r.torque?'保持中':'已卸力'}`);text('sequence-selected-detail',`ID ${selected} · 编码 ${r.position} · ${r.torque?'保持中':'已卸力'}`);}
    const receipts=IDS.map(id=>{const row=c.saved?.[id];return `ID ${id} ${c.unknown?.[id]?'待核对':row?(currentSaved.has(id)?'本次 ✓ 零位 ':'上次零位 ')+row.zero:'待校准'}`;}).join('\n');
    text('calibration-receipts',receipts);
    paintFlow();
  }
  window.DuckCalibration={action,stop:()=>action('stop'),isBusy,isMaintenance:maintenance,openFlow,closeFlow,snapshot:()=>data,select:id=>{selected=id;paint();},paint};
  get();setInterval(()=>{if(!document.hidden)get();},700);
})();
