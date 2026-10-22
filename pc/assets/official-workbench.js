/* UI adapter for the tested official API. No device write on load/reconnect. */
(() => {
  'use strict';
  const $=id=>document.getElementById(id), IDS=[10,11,12,13,14,20,21,22,23,24,30,31,32,33,34];
  let last=null,page='imu',owned=false,busy=false,commandBusy=false,selected=10,stoppedAt=null,actionGeneration=0,ownedForwardSpeed=.2,controlledMode=null,rollPending=false,rollSeen=false,statusErrorMessage=null,sessionPending=null,feedbackGapAt=null,lastCommandAckAt=null;
  const keyboard=window.DuckKeyboard.create();
  const STAND_ONLY=window.XGO_RELEASE_PROFILE?.schema==='xgoduck-stand-getup-publication-profile-v1';
  const BASIC=window.XGO_RELEASE_PROFILE?.schema==='xgoduck-basic-keyboard-publication-v2';
  const BASIC_MODEL_SHA='8a462e51d41fd43cbca1026698a2320f76fd84ef61d5c081cc63b7bcad7e394f';
  const labels={off:'已卸力',shadow:'只推理',hold:'校准中位保持',policy:'策略遥控',servo_debug:'舵机维护'};
  const text=(id,value)=>{if($(id)&&$(id).textContent!==value)$(id).textContent=value;};
  function statusItem(id,label,value,detail,state='unknown'){
    const item=$(id);if(!item)return;
    for(const [selector,content] of [['label',label],['strong',value],['small',detail]]){
      const child=item.querySelector(selector);if(child&&child.textContent!==String(content))child.textContent=content;
    }
    if(item.dataset.state!==state)item.dataset.state=state;
  }
  async function api(path,body){
    const r=await fetch(path,{cache:'no-store',signal:AbortSignal.timeout(3500),...(body===undefined?{}:{method:'POST',headers:{'Content-Type':'application/json','X-Token':window.COMMISSION_TOKEN},body:JSON.stringify(body)})});
    const value=await r.json();if(!r.ok)throw Error(value.detail||'请求未确认');return value;
  }
  const note=message=>{
    const el=$('wl-result');if(el.textContent===message)return;
    text('wl-result',message);
    if(typeof el.showPopover==='function'){
      if(el.matches?.(':popover-open'))el.hidePopover();
      if(message)el.showPopover();
    }
  };
  function sendPose(state){
    $('imu-duck-frame')?.contentWindow?.postMessage({type:'official-pose-state',version:1,status:state,frames:last?.runtime_frames,mode:document.hidden||page!=='imu'?'paused':state?'live':'stale',observedAtMs:Date.now()},location.origin);
    $('imu-duck-frame')?.contentWindow?.postMessage({type:'official-servo-current-state',version:1,current:state?.servo_current,mode:document.hidden||page!=='imu'?'paused':state?'live':'stale',observedAtMs:state?.pc_observed_unix_s*1000},location.origin);
    if(page==='models')$('models-frame')?.contentWindow?.postMessage({type:'bench-models-status',status:state,baseline:last?.baseline,connected:!!state&&!!last?.connected,mode:document.hidden?'paused':state&&last?.connected?'live':'stale'},location.origin);
  }
  function showPage(name){
    if(page!==name){setCalibrationVisible(false);setTestsVisible(false);}
    page=name;
    document.body.dataset.page=name;
    for(const p of ['imu','install','numbering','models'])$('workflow-'+p).hidden=p!==name;
    document.querySelectorAll('#workflow-tabs button').forEach(b=>{b.classList.toggle('active',b.dataset.page===name);if(b.dataset.page===name)b.setAttribute('aria-current','page');else b.removeAttribute('aria-current');});
    if(name==='install'&&!$('assembly-guide-frame').hasAttribute('src'))$('assembly-guide-frame').src='/assets/official-assembly.html';
    // Servo setup is official, lazily opened; changing pages releases its owner.
    if(name==='numbering'&&!$('numbering-frame').hasAttribute('src')){$('numbering-frame').src='/assets/numbering-restored.html';}
    $('numbering-frame')?.contentWindow?.postMessage({type:'duck-numbering-visibility',visible:name==='numbering'},location.origin);
    if(name==='models'&&!$('models-frame').hasAttribute('src'))$('models-frame').src='/assets/bench-models.html';
    $('models-frame')?.contentWindow?.postMessage({type:'bench-models-visibility',visible:name==='models'},location.origin);
    sendPose(last?.runtime);
  }
  document.querySelectorAll('#workflow-tabs button').forEach(b=>b.onclick=()=>showPage(b.dataset.page));
  $('diagnostics-open').onclick=()=>$('diagnostics-dialog').showModal();
  $('diagnostics-close').onclick=()=>$('diagnostics-dialog').close();
  $('diag-refresh').onclick=()=>poll();
  $('diag-export').disabled=false;
  $('diag-export').onclick=()=>{const link=document.createElement('a');link.href=URL.createObjectURL(new Blob([JSON.stringify(last,null,2)],{type:'application/json'}));link.download='xgoduck-workbench-status.json';link.click();URL.revokeObjectURL(link.href);};
  // This viewer reads the PC recorder only; it never requests servo reads or sends controls.
  const currentDialog=$('current-curve-dialog'),currentChannel=$('current-curve-channel');
  let currentSeries=null,currentTimer=null,currentRequest=null,currentGeneration=0;
  for(const id of IDS){const option=document.createElement('option');option.value=String(id);option.textContent='ID '+id;currentChannel.append(option);}
  function currentValue(sample){
    if(!sample)return null;
    const index=IDS.indexOf(Number(currentChannel.value));
    const value=index<0?(sample.mask===32767?sample.sum_abs_a:null):(sample.mask&(1<<index)?sample.values?.[index]:null);
    return Number.isFinite(value)?value:null;
  }
  function drawCurrentCurve(){
    const canvas=$('current-curve-canvas'),ctx=canvas.getContext('2d');if(!ctx)return;
    const samples=(currentSeries?.samples||[]).filter(sample=>Number.isFinite(sample.t));
    const finite=samples.map(currentValue).filter(Number.isFinite),latest=samples.at(-1),value=currentValue(latest);
    text('current-curve-value',value===null?'—':value.toFixed(3)+' A');
    text('current-curve-peak',finite.length?Math.max(...finite.map(Math.abs)).toFixed(3)+' A':'—');
    text('current-curve-hz',Number.isFinite(currentSeries?.sample_hz)?currentSeries.sample_hz.toFixed(1)+' Hz':'—');
    const w=canvas.width,h=canvas.height,left=66,right=w-22,top=24,bottom=h-42;
    ctx.clearRect(0,0,w,h);ctx.fillStyle='#09151f';ctx.fillRect(0,0,w,h);ctx.font='13px sans-serif';
    const max=finite.length?Math.max(.05,...finite):.5,min=finite.length?Math.min(0,...finite):0;
    const span=Math.max(.05,max-min),lo=min<0?min-span*.1:0,hi=max+span*.15;
    for(let i=0;i<=4;i++){
      const y=top+(bottom-top)*i/4;ctx.strokeStyle='#1b3544';ctx.beginPath();ctx.moveTo(left,y);ctx.lineTo(right,y);ctx.stroke();
      ctx.fillStyle='#8aa8b9';ctx.textAlign='right';ctx.fillText((hi-(hi-lo)*i/4).toFixed(2),left-10,y+4);
    }
    const end=latest?.t??Date.now()/1000,start=end-60,x=t=>left+(t-start)/60*(right-left),y=v=>bottom-(v-lo)/(hi-lo)*(bottom-top);
    for(let i=0;i<=4;i++){ctx.fillStyle='#8aa8b9';ctx.textAlign='center';ctx.fillText(i===4?'最新记录':`−${60-i*15} 秒`,left+(right-left)*i/4,h-14);}
    ctx.textAlign='left';ctx.fillText('A',16,17);
    ctx.strokeStyle='#55d7ed';ctx.lineWidth=1.7;ctx.beginPath();
    let previous=null;
    const gap=Math.max(.1,3/(currentSeries?.sample_hz||100));
    for(const sample of samples){
      const v=currentValue(sample);if(sample.t<start||sample.t>end||v===null){previous=null;continue;}
      const contiguous=previous&&sample.t>previous.t&&sample.t-previous.t<=gap&&(!Number.isFinite(sample.seq)||!Number.isFinite(previous.seq)||sample.seq===previous.seq+1);
      if(contiguous)ctx.lineTo(x(sample.t),y(v));else ctx.moveTo(x(sample.t),y(v));
      previous=sample;
    }
    ctx.stroke();
    if(!finite.length){ctx.fillStyle='#8aa8b9';ctx.textAlign='center';ctx.fillText('暂无完整电流样本',w/2,h/2);}
    canvas.setAttribute('aria-label',`${currentChannel.value==='sum'?'15颗合计':'ID '+currentChannel.value}电流，最近60秒，${samples.length}个记录，缺测处断开`);
  }
  function pauseCurrentCurve(){
    ++currentGeneration;if(currentTimer!==null)clearTimeout(currentTimer);currentTimer=null;
    currentRequest?.abort();currentRequest=null;
  }
  async function pollCurrentCurve(generation){
    if(generation!==currentGeneration||!currentDialog.open||document.hidden||currentRequest)return;
    const controller=new AbortController();currentRequest=controller;
    const timeout=setTimeout(()=>controller.abort(),3500);
    try{
      const response=await fetch('/api/bench/current-series',{cache:'no-store',signal:controller.signal});
      if(!response.ok)throw Error('读取失败');const data=await response.json();
      if(generation!==currentGeneration||!currentDialog.open||document.hidden)return;
      if(data.ok!==true||!Array.isArray(data.samples))throw Error('记录格式未确认');
      currentSeries=data;drawCurrentCurve();
      const latestTime=data.latest_recorded_unix_s??data.samples.at(-1)?.t,age=Number.isFinite(latestTime)?Math.max(0,Date.now()/1000-latestTime):null;
      text('current-curve-state',age===null?'暂无电流记录':age>1?`历史记录 · 最新样本距今 ${age.toFixed(1)} 秒`:'电脑记录更新中 · 只读查看');
      currentDialog.dataset.state=age!==null&&age<=1?'live':'history';
    }catch(error){
      if(generation!==currentGeneration||!currentDialog.open||document.hidden)return;
      text('current-curve-state',currentSeries?'连接中断 · 保留历史曲线，非实时':'暂时无法读取电流记录');currentDialog.dataset.state='history';
    }finally{
      clearTimeout(timeout);
      if(currentRequest===controller)currentRequest=null;
      if(generation===currentGeneration&&currentDialog.open&&!document.hidden)currentTimer=setTimeout(()=>pollCurrentCurve(generation),500);
    }
  }
  function resumeCurrentCurve(){
    pauseCurrentCurve();if(!currentDialog.open||document.hidden)return;
    text('current-curve-state',currentSeries?'正在更新 · 当前显示历史曲线':'读取电流记录中');
    currentDialog.dataset.state='history';pollCurrentCurve(currentGeneration);
  }
  function openCurrentCurve(){if(currentDialog.open)return;currentDialog.showModal();drawCurrentCurve();resumeCurrentCurve();}
  $('simple-current').onclick=openCurrentCurve;
  $('current-curve-open').onclick=openCurrentCurve;
  $('simple-current').onkeydown=e=>{if(e.key==='Enter'||e.key===' '){e.preventDefault();e.stopPropagation();openCurrentCurve();}};
  $('current-curve-close').onclick=()=>{pauseCurrentCurve();currentDialog.close();};
  currentDialog.onclose=pauseCurrentCurve;
  currentDialog.oncancel=()=>pauseCurrentCurve();
  currentChannel.value='sum';currentChannel.onchange=drawCurrentCurve;
  window.addEventListener('resize',()=>{if(currentDialog.open)drawCurrentCurve();});
  const trayDialog=$('calibration-tray-dialog'),trayFrame=$('calibration-tray-frame');
  function closeTrayPreview(){trayDialog.close();trayFrame.removeAttribute('src');}
  $('calibration-tray-open').onclick=e=>{e.preventDefault();if(trayDialog.open)return;trayFrame.setAttribute('src','/assets/calibration-tray.html');trayDialog.showModal();};
  $('calibration-tray-close').onclick=closeTrayPreview;
  trayDialog.onclose=()=>trayFrame.removeAttribute('src');
  window.addEventListener('message',e=>{if(e.origin===location.origin&&e.source===trayFrame.contentWindow&&e.data?.type==='duck-tray-escape'&&trayDialog.open){closeTrayPreview();stop();}});
  const calibrationDialog=$('calibration-flow-dialog');
  function setCalibrationVisible(visible){
    if(!visible&&window.DuckCalibration?.isBusy())return;
    $('sequence-panel').hidden=!visible;$('pose-calibration-layout').dataset.calibrating='false';$('sequence-toggle').setAttribute('aria-expanded',String(visible));
    if(visible){if(!calibrationDialog.open){calibrationDialog.showModal();window.DuckCalibration?.openFlow();}}
    else{if(calibrationDialog.open)calibrationDialog.close();window.DuckCalibration?.closeFlow();}
  }
  calibrationDialog.onclose=()=>{$('sequence-panel').hidden=true;$('sequence-toggle').setAttribute('aria-expanded','false');window.DuckCalibration?.closeFlow();};
  calibrationDialog.oncancel=e=>{if(window.DuckCalibration?.isBusy())e.preventDefault();};
  window.addEventListener('duck-calibration-close',()=>setCalibrationVisible(false));
  window.addEventListener('message',e=>{if(e.origin===location.origin&&e.source===$('calibration-flow-preview').contentWindow&&e.data?.type==='duck-tray-escape'&&calibrationDialog.open){if(!window.DuckCalibration?.isBusy())setCalibrationVisible(false);stop();}});
  function setTestsVisible(visible){const panel=$('bench-test-panel');panel.hidden=!visible;document.body.dataset.controlOpen=String(visible);$('bench-tests-toggle')?.setAttribute('aria-expanded',String(visible));if(visible){if(!panel.open)panel.show();}else if(panel.open)panel.close();}
  $('bench-test-panel').onclose=()=>{$('bench-test-panel').hidden=true;document.body.dataset.controlOpen='false';$('bench-tests-toggle')?.setAttribute('aria-expanded','false');};
  $('sequence-toggle').onclick=()=>{const visible=$('sequence-panel').hidden;setTestsVisible(false);setCalibrationVisible(visible);};
  $('calibration-panel-close').onclick=()=>setCalibrationVisible(false);
  $('bench-test-close').onclick=()=>setTestsVisible(false);
  $('sequence-passive-id10').hidden=true;
  text('sequence-hint','释放 → 手摆中位 → 校准并保持。其他已保持关节不变。');
  for(const id of IDS){const b=document.createElement('button');b.className='quiet';b.textContent='ID '+id;b.onclick=()=>selectJoint(id);$('sequence-joints').append(b);}
  document.querySelectorAll('[data-sequence-group]').forEach(b=>b.onclick=()=>highlight(b.dataset.sequenceGroup==='left'?IDS.slice(0,5):b.dataset.sequenceGroup==='right'?IDS.slice(5,10):IDS.slice(10)));
  const highlight=ids=>$('imu-duck-frame')?.contentWindow?.postMessage({type:'imu-model-highlight',version:1,ids},location.origin);
  function selectJoint(id){selected=id;window.dispatchEvent(new CustomEvent('duck-restored-joint',{detail:id}));highlight([id]);window.DuckCalibration?.select(id);paintJoint();}
  function paintJoint(){const i=IDS.indexOf(selected),s=last?.runtime;const position=s?.raw_pos?.[i],zero=s?.zero_pos?.[i];text('sequence-selected',`当前 ID ${selected}`);text('sequence-selected-detail',`ID ${selected} · 编码 ${position??'—'} · 零位 ${zero??'—'}`);window.DuckCalibration?.paint();}
  window.addEventListener('duck-calibration-select',e=>selectJoint(e.detail));
  $('wl-home').textContent='回正';
  function homeFeedbackFresh(){const s=last?.runtime;return last?.connected&&s?.full_robot_ready===true&&Number.isFinite(s.feedback_age_ms)&&s.feedback_age_ms>=0&&s.feedback_age_ms<150;}
  $('wl-home').onclick=()=>{
    if(!homeFeedbackFresh()||window.DuckCalibration?.snapshot()?.calibration?.active){note('反馈未确认或校准进行中，请等待当前状态。');return;}
    const target=last.runtime.policy_targets;
    if(last.runtime.mode==='hold'&&Array.isArray(target)&&target.length===15&&target.every(value=>Number.isFinite(value)&&value===0)){note('正在保持校准中位，无需重复回正');return;}
    const angles=last.runtime.angles;
    if(Array.isArray(angles)&&angles.length===15&&angles.every(value=>Number.isFinite(value)&&Math.abs(value)<=.35)){note('已在校准中位，无需回正');return;}
    prepare('hold');
  };
  $('wl-stop').onclick=()=>stop();
  const controls=document.createElement('span');controls.className='bench-controls';
  controls.innerHTML='<button id="bench-tests-toggle" class="quiet" aria-haspopup="dialog" aria-controls="bench-test-panel" aria-expanded="false">控制</button>';
  $('sequence-toggle').before(controls);
  const actions=document.createElement('div');actions.className='bench-actions';
  actions.innerHTML='<button id="bench-policy">站立 / 起身</button><button id="bench-control-stop" class="danger">停止 / 卸力</button>';
  $('bench-test-panel').querySelector('p').after(actions);
  const extraActions=document.createElement('div');extraActions.className='bench-actions';
  extraActions.innerHTML='<button id="bench-pick" class="quiet">空载 Pick</button><button id="bench-roulade" class="quiet">翻滚模型</button><label class="bench-forward-speed">W 前进<select id="bench-forward-speed" aria-label="W 前进速度"><option value="0.2" selected>0.2 m/s</option></select></label>';
  $('bench-more-tests-content').querySelector('p').after(extraActions);
  const keyboardPanel=document.createElement('section');keyboardPanel.className='bench-keyboard-panel';
  keyboardPanel.innerHTML=`<div class="control-guide"><div class="control-guide-group"><strong>行走 / 动作${STAND_ONLY?' · 当前停用':''}</strong><div class="control-wasd"><kbd class="control-key key-w" data-key="w">W<small>前进</small></kbd><kbd class="control-key key-a" data-key="a">A<small>左转</small></kbd><kbd class="control-key key-s" data-key="s">S<small>后退</small></kbd><kbd class="control-key key-d" data-key="d">D<small>右转</small></kbd></div><div class="control-wide-keys"><kbd class="control-key" data-key="shift">Shift<small>快跑</small></kbd><kbd class="control-key" data-key="space">Space<small>翻滚</small></kbd></div></div><div class="control-guide-group"><strong>头部 / 嘴部 · 松键保持</strong><div class="control-head-keys"><kbd class="control-key" data-key="i">I<small>抬头</small></kbd><kbd class="control-key" data-key="k">K<small>${STAND_ONLY?'回中':'低头'}</small></kbd><kbd class="control-key" data-key="j">J<small>左看</small></kbd><kbd class="control-key" data-key="l">L<small>右看</small></kbd><kbd class="control-key" data-key="u">U<small>张嘴</small></kbd><kbd class="control-key" data-key="o">O<small>闭嘴</small></kbd></div></div></div><p>启动站立后，头嘴逐次累加，松键保持。Esc 停止卸力。</p><p id="bench-keyboard-state">尚未启动</p><button id="bench-head-center" class="quiet" disabled>头部回中</button>`;
  actions.after(keyboardPanel);
  const keyboardDetails=document.createElement('div');keyboardDetails.className='bench-keyboard-details';
  keyboardDetails.innerHTML='<output id="bench-keyboard-readout" aria-live="polite"></output><p>W 前进 0.2 m/s · Shift + W 快跑 0.3 m/s · S 后退 · A / D 左右转 · Space 翻滚 · Esc 停止卸力。</p><small>净转向优先，走动松键停止。K沿Pick姿态逐步低头，I逐步抬回；每次累加，松键保持。专用运动模型期间头部指令累计，回原站立后应用。历史翻滚实物未通过。</small>';
  $('bench-more-tests-content').append(keyboardDetails);
  if(STAND_ONLY){keyboardPanel.querySelector('p').textContent='启动站立后可控制头嘴；行走、快跑与翻滚当前停用。Esc 停止卸力。';keyboardDetails.querySelector('p').textContent='当前仅原版零速站立 / 起身；行走、快跑、深度Pick、翻滚默认停用。Esc停止卸力。';keyboardDetails.querySelector('small').textContent='头嘴逐档累加、松键保持；K退回HOME，不进入深度Pick。翻滚断电原因未修复，相关入口保留但停用。';}
  if(BASIC){keyboardPanel.querySelector('.control-guide-group:last-child strong').textContent='头部 / 嘴部 · 本轮停用';keyboardPanel.querySelector('p').textContent='本轮基础测试：W/S/A/D走动，Shift+W加速；松键立即发送零速度。头嘴、深低头和翻滚尚未验收，相关键暂时停用。Esc停止卸力。';keyboardDetails.querySelector('p').textContent='同一模型：W前进0.2、Shift+W前进0.3、S后退−0.3 m/s，A/D左右转±1 rad/s；松键零速。';keyboardDetails.querySelector('small').textContent='全程同一actor，没有旧版3秒刹车或模型互换。JKLUIO、Space、头部回中、Pick本轮停用；基础行走实物效果仍待验收。';}
  $('bench-control-stop').onclick=()=>stop();
  $('bench-policy').onclick=()=>prepare('policy');$('bench-pick').onclick=()=>prepare('pick');$('bench-roulade').onclick=()=>prepare('roulade');
  $('bench-tests-toggle').onclick=()=>{const visible=$('bench-test-panel').hidden;setCalibrationVisible(false);setTestsVisible(visible);};
  const dialog=document.createElement('dialog');dialog.id='bench-motion-dialog';dialog.innerHTML='<h2>动作预告</h2><p id="bench-motion-description"></p><p>放好机身、留出动作空间，手离开后启动。停止按钮随时卸力。</p><div class="bench-actions"><button id="bench-cancel" class="quiet">取消</button><button id="bench-confirm">已准备好，启动本次动作</button></div>';
  document.body.append(dialog);let pending=null;
  $('bench-cancel').onclick=()=>dialog.close();
  function forwardSpeed(){return .2;}
  function keyboardCapabilitiesReady(){if(STAND_ONLY)return true;const basis=last?.baseline;if(BASIC)return basis?.basic_keyboard_policy?.active===true&&basis.basic_keyboard_policy.sha256===BASIC_MODEL_SHA&&basis?.walk_policy?.sha256===BASIC_MODEL_SHA;return basis?.fast_policy?.active===true&&basis?.navigation_policy?.active===true&&basis?.manual_pick?.active===true;}
  function paintSpeed(){
    if(STAND_ONLY){const selector=$('bench-forward-speed');selector.value='0.2';selector.disabled=true;selector.title='本次发布仅零速站立/起身；行走与快跑停用。';for(const option of selector.options)option.disabled=true;if(selector.options[0])selector.options[0].textContent='行走 / 快跑默认停用';return;}
    if(BASIC){const selector=$('bench-forward-speed');selector.value='0.2';selector.disabled=true;if(selector.options[0])selector.options[0].textContent='同一模型 0.2 m/s';selector.title='W 0.2、Shift+W 0.3；前后左右与松键零速均使用同一actor，实物待验收。';return;}
    const selector=$('bench-forward-speed'),available=last?.baseline?.fast_policy?.active===true;selector.value='0.2';
    for(const option of selector.options)option.disabled=Number(option.value)!==.2;
    if(selector.options[0])selector.options[0].textContent=available?'新模型 0.2 m/s':'原版 0.2 m/s（新模型尚未部署）';
    selector.disabled=owned;selector.title=available?'纯 W 使用前进模型 0.2 m/s，Shift+W 同模型 0.3 m/s；S/A/D使用已准入导航模型，转向优先。':'键盘控制升级待部署，当前不启动键盘遥控。';
  }
  $('bench-forward-speed').onchange=()=>{if(!owned){$('bench-forward-speed').value=String(forwardSpeed());}else{$('bench-forward-speed').value=String(ownedForwardSpeed);}};
  function manualReady(){const s=last?.runtime;return keyboardCapabilitiesReady()&&owned&&!sessionPending&&controlledMode==='policy'&&last?.connected&&s?.full_robot_ready===true&&s?.mode==='policy'&&s?.recovery?.phase==='walk'&&!s?.pick?.active&&!s?.roulade?.active&&!rollPending;}
  const recovering=()=>controlledMode==='policy'&&last?.runtime?.mode==='policy'&&!!last.runtime.recovery?.phase&&last.runtime.recovery.phase!=='walk';
  function clearRecoveryIntent(){if(recovering()){keyboard.clearMotion();keyboard.centerHead();}}
  function automaticReady(){if(STAND_ONLY||BASIC)return false;const s=last?.runtime,forward=s?.forward_specialist||s?.fast_forward,navigation=s?.navigation_specialist,motionHeld=keyboard.snapshot().held.some(key=>['w','s','a','d'].includes(key));return manualReady()&&!motionHeld&&!forward?.active&&!forward?.braking&&!navigation?.active&&!navigation?.braking&&keyboard.snapshot().manualPickDepth===0&&!s?.manual_pick?.active&&!(s?.manual_pick?.target_depth>0);}
  function paintKeyboard(){
    const value=keyboard.snapshot(),degrees=v=>(v*180/Math.PI).toFixed(1),manual=last?.runtime?.manual_pick,forward=last?.runtime?.forward_specialist||last?.runtime?.fast_forward,navigation=last?.runtime?.navigation_specialist;
    document.querySelectorAll('.control-key').forEach(key=>{const blocked=(STAND_ONLY&&['w','a','s','d','shift','space'].includes(key.dataset.key))||(BASIC&&['j','k','l','u','i','o','space'].includes(key.dataset.key));key.setAttribute('aria-disabled',String(blocked||!owned));key.dataset.pressed=String(!blocked&&owned&&value.held.includes(key.dataset.key));if(BASIC)key.style.opacity=blocked?'.45':'';});
    const phase=Number.isFinite(manual?.depth)?`${Math.round(manual.depth*100)}%${manual.returning?'（抬回中）':''}`:'待读取';
    const deferred=navigation?.active||navigation?.braking?' · 导航模型期间头部指令已累计，回原站立后应用':forward?.active||forward?.braking?' · 前进模型期间头部指令已累计，回原版行走后应用':manual?.active?' · 左右看抬回后生效':'';
    text('bench-keyboard-readout',`低头目标阶段 ${Math.round(value.manualPickDepth*100)}% · 主控当前阶段 ${phase} · 抬头指令 ${degrees(Math.max(0,-value.pitchLevel))}° · 左右 ${degrees(value.head[2])}° · 嘴 ${value.mouth.toFixed(0)}° · 松键保持${deferred}`);
    if(BASIC)text('bench-keyboard-readout','基础键盘测试 · 头部、嘴部和深低头指令固定为零；松键立即发零速，实物停稳待验收。');
    $('bench-head-center').disabled=BASIC||!manualReady();
    text('bench-keyboard-state',!keyboardCapabilitiesReady()?'键盘控制升级待部署；回正、校准与停止入口保留。':!owned?'尚未启动，键盘不会动作。':sessionPending?'启动确认中 · 保持零指令心跳，主控确认后才开放按键。':last?.runtime?.pick?.active?'官方 Pick 进行中，头嘴由模型控制。':last?.runtime?.roulade?.active||rollPending?'翻滚请求 / 动作中；历史实物翻滚未通过。':navigation?.braking?`导航退出：主控零速刹停中，剩余 ${Number(navigation.brake_remaining_s||0).toFixed(1)} s；松键后不续走。`:forward?.braking?`前进退出：主控零速度刹停中，剩余 ${Number(forward.brake_remaining_s||0).toFixed(1)} s；行走松键后目标为零，不自动续走。`:STAND_ONLY?'站立保持中 · 头嘴松键保持。':'本页已启动 · W 0.2 / Shift+W 0.3 · S/A/D导航纯点、转向优先（实物待验收）；K低头 / I抬回，松键保持。');
    if(BASIC&&owned&&!sessionPending)text('bench-keyboard-state',recovering()?'原版起身中，保持零指令；站稳后重新按键。新版行走与起身交接尚待实物验收。':'同一actor基础控制 · W .2 / Shift+W .3 / S −.3 / A/D ±1；松键零速。头嘴/翻滚本轮停用，实物效果待验收。');
  }
  function loseOwnership(){++actionGeneration;owned=false;controlledMode=null;sessionPending=null;feedbackGapAt=null;lastCommandAckAt=null;keyboard.reset();rollPending=false;rollSeen=false;paintSpeed();paintKeyboard();}
  $('bench-head-center').onclick=()=>{if(BASIC||!manualReady())return;keyboard.centerHead();paintKeyboard();command();};
  async function triggerRoulade(){
    if(!automaticReady()){note('先松开走动键，停稳后再按空格。');return false;}
    rollPending=true;rollSeen=false;keyboard.clearMotion();paintKeyboard();
    try{await api('/api/roulade',{});note('已请求一次官方翻滚；历史实物翻滚未通过，等待本次实际结果。');return true;}
    catch(e){loseOwnership();note('翻滚请求未确认，遥控已取消：'+e.message);return false;}
  }
  function prepare(action){
    if(action==='policy'&&!keyboardCapabilitiesReady()){pending=null;note('键盘控制升级待部署；当前不会启动策略或发送新协议。');paintKeyboard();return;}
    if((action==='pick'||action==='roulade')&&!automaticReady()){pending=null;note('先松开走动键，停稳后再启动动作。');return;}
    pending={action,forwardSpeed:forwardSpeed()};
    text('bench-motion-description',{hold:'回到已保存的校准中位并保持。',policy:'将进入站立 / 跌倒起身策略并启用本页键盘，起步零速度。W用前进策略0.2 m/s，Shift+W同策略0.3；S用原模型后退指令0.3（仿真实测约0.18–0.22 m/s），A/D用原模型原地左右转1 rad/s。有净AD优先转向，无净转向再按净WS前后，相反键抵消，走动松键停止。专用模型互换先当前零速刹停3秒、原站立一帧，再执行仍按住的最新方向；头部先累计，回原站立后应用，嘴即时。K/I按档低头抬回、松键保持，腿配合；J/L左右看（低头时延后），U/O张闭嘴；走动前先抬回。Space请求一次官方翻滚（历史实物未通过），Esc停止卸力。所有运动实物待验收。',pick:'将执行一次空载低头开合嘴动作，结束后回到站立策略。',roulade:'将执行一次官方翻滚模型。历史实物翻滚未通过，不预设本次成功。'}[action]);
    if(STAND_ONLY&&action==='policy')text('bench-motion-description','将启动原版零速站立/必要起身并保持。普通抬头、左右看与嘴部原接口保留；行走/快跑/深度Pick/翻滚默认停用，翻滚断电原因尚未修复。');
    if(BASIC&&action==='policy')text('bench-motion-description','将启用新版基础键盘模型与原版必要起身，初始零速度。W前进0.2、Shift+W前进0.3、S后退−0.3 m/s，A/D原地左右转±1 rad/s；净转向优先，相反键抵消，松键立即发送零速。同一actor连续运行，没有旧版3秒刹车。JKLUIO、Space、Pick和头部回中本轮停用。实物行走/停稳及起身交接尚待验收。');
    dialog.showModal();
  }
  async function leaveCalibrationForHome(generation){
    const deadline=performance.now()+5000;
    await api('/api/bench/calibration/action',{action:'restore',id:selected,confirmed:true});
    while(generation===actionGeneration&&performance.now()<deadline){
      const response=await fetch('/api/bench/calibration',{cache:'no-store',signal:AbortSignal.timeout(Math.max(1,Math.min(3500,deadline-performance.now())))});
      const state=await response.json();if(!response.ok)throw Error(state.detail||'退出校准未确认');
      if(generation!==actionGeneration)throw Error('回正已取消');
      if(state.calibration?.error)throw Error(state.calibration.error);
      if(state.calibration?.active===false&&state.maintenance?.active===false)return;
      await new Promise(resolve=>setTimeout(resolve,Math.min(200,Math.max(0,deadline-performance.now()))));
    }
    throw Error(generation!==actionGeneration?'回正已取消':'5秒内未确认退出校准，未启动回正');
  }
  $('bench-confirm').onclick=async()=>{
    const action=pending?.action,speed=pending?.forwardSpeed,generation=++actionGeneration;dialog.close();keyboard.releaseAll();
    const priorUptime=last?.runtime?.uptime_s,priorEpoch=last?.runtime?.pc_host_epoch;
    if(!action)return;
    if(action==='policy'&&!keyboardCapabilitiesReady()){pending=null;note('键盘控制升级待部署；未发送策略启动请求。');paintKeyboard();return;}
    try{
      if(action==='hold'||action==='policy'){
        if(action==='hold'&&window.DuckCalibration?.snapshot()?.maintenance?.active){
          owned=false;controlledMode=null;sessionPending=null;
          note('正在退出校准，确认后回到校准中位。');await leaveCalibrationForHome(generation);
          if(generation!==actionGeneration)return;
        }
        await api('/api/mode',{mode:action});
        if(generation!==actionGeneration){await api('/api/mode',{mode:'off'});return;}
        keyboard.reset();rollPending=false;rollSeen=false;ownedForwardSpeed=.2;owned=true;controlledMode=action;
        // Status streaming can still contain OFF from before the accepted POST.
        // Keep only zero heartbeats until a newer matching snapshot confirms it.
        feedbackGapAt=null;sessionPending={mode:action,priorUptime,priorEpoch,deadline:performance.now()+3500};
        paintSpeed();paintKeyboard();await command();
      }
      else{if(!automaticReady()){note('先松开走动键，停稳后再启动动作。');return;}if(action==='roulade'){if(!await triggerRoulade())return;}else await api('/api/'+action,{});}
      if(generation!==actionGeneration)return;
      stoppedAt=null;note(action==='hold'||action==='policy'?(sessionPending?'启动请求已接收，保持零指令并等待主控确认。':action==='hold'?'主控已确认回正，正在保持校准中位。':'主控已确认启动，键盘控制已就绪。'):'动作请求已发出，等待主控状态确认。');
    }catch(e){if(generation!==actionGeneration)return;loseOwnership();note(e.message);}
  };
  async function stop(){
    loseOwnership();stoppedAt=Date.now();note('停止请求中，等待 OFF 与 MCU 卸力回执。');
    if(window.DuckCalibration?.isMaintenance()){await window.DuckCalibration.stop();return;}
    try{await api('/api/mode',{mode:'off'});await poll();}catch(e){note('停止结果未确认：'+e.message);}
  }
  async function command(){
    if(!owned||commandBusy||document.hidden)return;
    if(controlledMode==='policy'&&!keyboardCapabilitiesReady()){loseOwnership();note('键盘控制升级待部署；新协议发送已暂停。');return;}
    clearRecoveryIntent();
    commandBusy=true;const generation=actionGeneration;
    // Same manual command units and endpoint as the official browser. The policy creates joint targets.
    const packet=controlledMode==='hold'?{twist:[0,0,0],head:[0,0,0,0],mouth:0}:sessionPending||feedbackGapAt!==null?{twist:[0,0,0],head:[0,0,0,0],mouth:0,manual_pick_depth:0,forward_specialist:false,fast_forward:false,navigation_specialist:false}:keyboard.packet({fastAllowed:last?.baseline?.fast_policy?.active===true,navAllowed:last?.baseline?.navigation_policy?.active===true,pickActive:!!last?.runtime?.pick?.active,pickMouth:last?.runtime?.mouth});
    if(STAND_ONLY){packet.twist=[0,0,0];packet.manual_pick_depth=0;packet.forward_specialist=false;packet.navigation_specialist=false;packet.fast_forward=false;}
    if(BASIC){packet.head=[0,0,0,0];packet.mouth=0;packet.manual_pick_depth=0;}
    try{await api('/api/command',packet);if(generation===actionGeneration&&owned)lastCommandAckAt=performance.now();}
    catch(e){if(generation!==actionGeneration)return;loseOwnership();note('控制通信中断；不重发、不恢复：'+e.message);}finally{commandBusy=false;}
  }
  window.addEventListener('keydown',e=>{
    if(window.DuckKeyboard.keyName(e)==='space'&&$('bench-test-panel').open&&!/INPUT|SELECT|TEXTAREA/.test(e.target.tagName)&&!e.target.isContentEditable&&!dialog.open&&!calibrationDialog.open){
      e.preventDefault();
      if(!owned||sessionPending||controlledMode!=='policy'){note('尚未启动，空格不会动作。');return;}
    }
    if(e.key==='Escape'){e.preventDefault();if(e.repeat)return;if(currentDialog.open){pauseCurrentCurve();currentDialog.close();}if(trayDialog.open)closeTrayPreview();if(calibrationDialog.open&&!window.DuckCalibration?.isBusy())setCalibrationVisible(false);if($('bench-test-panel').open)setTestsVisible(false);dialog.close();stop();return;}
    if(/INPUT|SELECT|TEXTAREA/.test(e.target.tagName)||e.target.isContentEditable||dialog.open||currentDialog.open||trayDialog.open||calibrationDialog.open||!owned||sessionPending||controlledMode!=='policy')return;
    if(recovering()){e.preventDefault();clearRecoveryIntent();paintKeyboard();note('正在起身，保持零指令；站稳后需重新按键。');return;}
    const publicationKey=window.DuckKeyboard.keyName(e);
    if(BASIC&&['j','k','l','u','i','o','space'].includes(publicationKey)){e.preventDefault();note('本轮只测基础行走与停止；头嘴、深低头和翻滚尚未验收，暂时停用。');return;}
    if(STAND_ONLY&&(['w','s','a','d','shift','space'].includes(publicationKey)||publicationKey==='k'&&keyboard.snapshot().pitchLevel>=-1e-9)){e.preventDefault();note('发布配置默认停用移步/快跑/深度Pick/翻滚；普通抬头、左右看与头嘴控制保留。');return;}
    if(e.repeat&&!keyboard.snapshot().held.includes(window.DuckKeyboard.keyName(e))){e.preventDefault();return;}
    if(window.DuckKeyboard.keyName(e)==='space'&&!automaticReady()){e.preventDefault();note('先松开走动键，停稳后再按空格。');return;}
    const outcome=keyboard.keydown(e,{allowHead:!BASIC&&manualReady()});if(!outcome.handled)return;e.preventDefault();paintKeyboard();
    if(outcome.roll){triggerRoulade();return;}if(outcome.changed)command();
  });
  window.addEventListener('keyup',e=>{if(window.DuckKeyboard.keyName(e)==='space'&&$('bench-test-panel').open&&!/INPUT|SELECT|TEXTAREA/.test(e.target.tagName)&&!e.target.isContentEditable)e.preventDefault();if(keyboard.keyup(e)){paintKeyboard();if(['w','s','a','d','shift'].includes(window.DuckKeyboard.keyName(e)))command();}});
  window.addEventListener('blur',()=>{keyboard.releaseAll();paintKeyboard();command();});
  document.addEventListener('visibilitychange',()=>{if(document.hidden&&owned)stop();sendPose(document.hidden?null:last?.runtime);if(document.hidden){pauseCurrentCurve();if(currentDialog.open){text('current-curve-state','页面已隐藏 · 保留历史曲线');currentDialog.dataset.state='history';}}else if(currentDialog.open)resumeCurrentCurve();});
  window.addEventListener('pagehide',()=>{if(owned)navigator.sendBeacon('/api/mode',new Blob([JSON.stringify({mode:'off'})],{type:'application/json'}));});
  setInterval(command,50);
  $('bench-test-start').onclick=async()=>{try{await api('/api/bench/tests/start',{profile:$('bench-test-profile').value,duration_s:Number($('bench-test-duration').value)});await poll();}catch(e){text('bench-test-result',e.message);}};
  $('bench-test-stop').onclick=async()=>{try{await api('/api/bench/tests/stop',{});await poll();}catch(e){text('bench-test-result',e.message);}};
  $('bench-test-annotate').onclick=async()=>{try{await api('/api/bench/tests/annotate',{physical:$('bench-physical').value,note:$('bench-physical-note').value});await poll();}catch(e){text('bench-test-result',e.message);}};
  function paintTest(t){
    if(!t)return;
    text('bench-test-result',!t.run_id&&!t.id?'尚未开始记录':`${t.active?'记录中':'记录结束'} · ${t.reason||t.digital_status||'观察中'} · 最长站稳 ${Number(t.longest_stable_s||0).toFixed(1)} s`);
    $('bench-test-start').disabled=!!t.active||!last?.connected;
    $('bench-test-stop').disabled=!t.active;
    $('bench-test-annotate').disabled=!!t.active||!(t.run_id||t.id);
    const run=t.run_id||t.id;if(run){$('bench-report').href='/api/bench/report/'+encodeURIComponent(run)+'/result.json';$('bench-report').hidden=false;}
  }
  async function handleFeedbackGap(fresh){
    if(owned&&!fresh&&!sessionPending){
      const now=performance.now();
      if(feedbackGapAt===null)feedbackGapAt=now;
      keyboard.releaseAll();
      // Display receipt and command acknowledgement are separate channels.
      // Continue zero heartbeats when commands are still acknowledged; the
      // controller retains its own 1s command watchdog and local sensor guard.
      const commandsAcknowledged=lastCommandAckAt!==null&&now-lastCommandAckAt<750;
      note(commandsAcknowledged?'姿态显示延迟，控制回执正常；暂发零速度，等待新姿态。':'正在核对控制回执，暂时保持零速度。');
      if(!commandsAcknowledged&&now-feedbackGapAt>=500){await stop();note('控制与姿态回执均中断，已请求停止卸力。');return true;}
    }else if(fresh&&feedbackGapAt!==null){feedbackGapAt=null;keyboard.releaseAll();note('主控反馈已恢复，请重新按键。');}
    return false;
  }
  async function poll(){
    if(busy)return;busy=true;const generation=actionGeneration;
    try{
      const observation=last?.runtime?.pc_observation_id;
      const cursor=Number.isSafeInteger(observation)?'?after_observation='+encodeURIComponent(observation):'';
      const snapshot=await api('/api/bench/status'+cursor);if(generation!==actionGeneration)return;
      last=snapshot;window.dispatchEvent(new CustomEvent('duck-restored-status',{detail:last}));const s=last.runtime;
      if(statusErrorMessage&&$('wl-result').textContent===statusErrorMessage)note('');statusErrorMessage=null;
      const fresh=last.connected&&s?.full_robot_ready===true;
      if(owned&&sessionPending&&(controlledMode!=='policy'||keyboardCapabilitiesReady())){
        const newer=(!Number.isFinite(sessionPending.priorUptime)||Number.isFinite(s?.uptime_s)&&s.uptime_s>sessionPending.priorUptime)&&
          (sessionPending.priorEpoch===undefined||s?.pc_host_epoch===sessionPending.priorEpoch);
        if(fresh&&s?.mode===sessionPending.mode&&s.enabled===true&&newer){sessionPending=null;keyboard.reset();note('主控已确认启动，键盘控制已就绪。');}
        else if(performance.now()>=sessionPending.deadline){await stop();note('启动状态未在3.5秒内确认，已请求停止卸力；请核对当前状态。');return;}
      }
      statusItem('simple-link','无线',last.connected?'已连接':last.wireless_transport_alive?'状态延迟':'未连接','官方运行链',last.connected?'ok':last.wireless_transport_alive?'pending':'error');
      const servoCount=new Set((s?.servo_ids||[]).filter(id=>IDS.includes(id))).size;
      statusItem('simple-servos','主控舵机',s?servoCount+' / 15':'— / 15','真实反馈',!last.connected?'unknown':servoCount===15?'ok':'error');
      statusItem('simple-imu','IMU',s?.imu_ok?'在线':'待连接','主控官方重力估计',!last.connected?'unknown':s?.imu_ok?'ok':'error');
      const torqueKnown=typeof s?.enabled==='boolean';
      statusItem('simple-motion','主控运动',torqueKnown?labels[s.mode]||s.mode:'待读取',torqueKnown?(s.enabled?'上力反馈':'卸力反馈'):'未收到主控反馈',!last.connected||!torqueKnown?'unknown':s.enabled?'active':s.mode==='off'?'ok':'pending');
      const maintenanceView=window.DuckCalibration?.snapshot();
      if(maintenanceView?.maintenance?.active&&!maintenanceView.maintenance.integrated){
        const calibration=maintenanceView.calibration,phase=maintenanceView.maintenance.phase;
        statusItem('simple-link','主控','厂家维护','正式模型已暂停','pending');
        statusItem('simple-servos','本次校准',Object.keys(calibration.saved||{}).length+' / 15','已保存回执','pending');
        statusItem('simple-imu','IMU','维护暂停','未显示实时姿态','unknown');
        statusItem('simple-motion','维护状态',phase==='maintenance'?'当前状态待读回':phase==='unknown'?'结果待核对':'切换中','上次校准回执不代表当前扭矩','pending');
      }
      const power=s?.servo_power,voltage=power?.voltage_v,known=Number.isFinite(voltage)&&voltage>0&&Number.isFinite(power?.age_ms)&&power.age_ms>=0;
      const powerFresh=last.connected&&power?.available===true&&known&&power.age_ms<5000;
      text('battery-percent',known?voltage.toFixed(1)+' V'+(powerFresh?'':' · 上次'):'待读取');
      text('battery-voltage',known?`ID ${power.servo_id} · ${(power.age_ms/1000).toFixed(1)} 秒前 · 舵机总线`:'等待舵机电压反馈');
      $('simple-battery').dataset.state=powerFresh?'ok':known?'pending':'unknown';
      $('simple-battery').title=known?`舵机 ID ${power.servo_id} 的总线电压；${(power.age_ms/1000).toFixed(1)} 秒前采集，低频读取，不代表电池电量百分比。`:'等待低频舵机电压采集';
      const current=s?.servo_current,currentKnown=last.connected&&current?.available===true&&Number.isFinite(current?.sum_abs_a),recording=last.current_recording;
      statusItem('simple-current','舵机电流合计',currentKnown?current.sum_abs_a.toFixed(3)+' A':current?.supported?'本帧未齐':'待读取',
        `${Number.isFinite(current?.source_hz)?current.source_hz.toFixed(1)+' Hz':'频率待确认'} · ${recording?.recording?'电脑记录中':current?.logging?'主控记录中':'等待记录'}`,
        currentKnown?'ok':current?.supported?'pending':'unknown');
      text('wl-home-state',s?`${labels[s.mode]||s.mode} · ${s.recovery?.phase||'—'} · 倾角 ${Number.isFinite(s.recovery?.angle_deg)?s.recovery.angle_deg.toFixed(1)+'°':'—'} · 反馈 ${s.feedback_hz?.toFixed(1)??'—'} Hz`:'主控无线未连接；页面保留，连接后只读恢复。');
      if(maintenanceView?.maintenance?.active&&!maintenanceView.maintenance.integrated)text('wl-home-state','厂家校准维护中，正式模型与实时姿态显示暂停。当前扭矩状态需要重新读回。');
      $('wl-home').disabled=!homeFeedbackFresh()||!last.runtime_basis_matches||!!maintenanceView?.calibration?.active;
      $('bench-policy').disabled=$('wl-home').disabled||!keyboardCapabilitiesReady();
      $('bench-pick').disabled=$('bench-roulade').disabled=!fresh||!automaticReady();
      if(await handleFeedbackGap(fresh))return;
      if(owned&&(fresh&&!sessionPending&&(s?.mode==='off'||s?.mode==='shadow')||controlledMode==='policy'&&!keyboardCapabilitiesReady())){
        const reason=s?.last_error;loseOwnership();
        note(reason?(reason==='web heartbeat lost; disarmed'?'网页控制心跳中断，主控已保护卸力；需要重新明确启动。':'主控已暂停控制：'+reason):!fresh?'主控反馈未确认，键盘控制已暂停。':'主控已退出上力控制，键盘已暂停；需要重新明确启动。');
      }
      clearRecoveryIntent();
      if(rollPending&&s?.roulade?.active)rollSeen=true;
      if(rollPending&&rollSeen&&!s?.roulade?.active){rollPending=false;rollSeen=false;}
      if(stoppedAt&&s?.mode==='off'&&s.enabled===false&&s.feedback_age_ms<150&&s.mcu_command_age_us<150000){note('停止已确认：OFF、disabled、MCU 新鲜卸力回执。');stoppedAt=null;}
      paintSpeed();text('simple-pending',keyboardCapabilitiesReady()?'基线：前进0.2/0.3 + 导航纯后退/原地转向 + 手动Pick；净转向优先，实物待验收。':'键盘控制升级待部署；原回正、校准与停止入口保留。');
      if(BASIC&&keyboardCapabilitiesReady())text('simple-pending','新版基础模型：前后左右与松键零速均为同一actor；头嘴、Pick、翻滚本轮停用，实物验收待进行。');
      text('diag-service','工作台 '+last.revision);text('diag-controller',last.connected?'已连接':'未连接');text('diag-torque',torqueKnown?(s.enabled?'上力':'卸力'):'未知');
      text('diag-identity',last.runtime_basis_matches?'新零位 / 直接起身参数匹配':'当前主控基线待读取或不匹配');text('diag-model',s?.active_model||'模型尚未运行');text('diag-refresh-note',last.error||`本地源文件 ${last.baseline.files_checked} 项核对：${last.baseline.local_sources_verified?'通过':'失败'}；主控源哈希另行验收。`);
      text('comm-version','基于 '+last.baseline.reference.slice(0,12));text('comm-result','安装 / 更新继续使用已验证的专项流程。');
      for(const id of ['comm-usb','comm-update','comm-discover'])$(id).disabled=true;
      paintJoint();paintTest(last.test);paintKeyboard();sendPose(s);
    }catch(e){if(generation!==actionGeneration)return;loseOwnership();sendPose(null);statusItem('simple-link','无线','待读取','当前连接未确认','error');statusItem('simple-servos','主控舵机','— / 15','当前反馈未确认');statusItem('simple-imu','IMU','未知','当前反馈未确认');statusItem('simple-motion','主控运动','未知','当前扭矩未确认');text('battery-percent','待读取');$('simple-battery').dataset.state='unknown';text('diag-controller','待读取');text('diag-torque','未知');for(const id of ['wl-home','bench-policy','bench-pick','bench-roulade'])$(id).disabled=true;statusErrorMessage='电脑服务读取失败：'+e.message;note(statusErrorMessage);}finally{busy=false;}
  }
  window.addEventListener('message',e=>{if(e.origin!==location.origin)return;if(e.source===$('imu-duck-frame')?.contentWindow){if(e.data?.type==='imu-model-ready')sendPose(last?.runtime);if(e.data?.type==='imu-current-curve-open')openCurrentCurve();if(e.data?.type==='imu-model-joint-selected')selectJoint(e.data.id);}if(e.source===$('models-frame')?.contentWindow){if(e.data?.type==='bench-models-ready')sendPose(last?.runtime);if(e.data?.type==='bench-stop-request')stop();if(e.data?.type==='bench-model-test'){showPage('imu');if([...$('bench-test-profile').options].some(o=>o.value===e.data.profile))$('bench-test-profile').value=e.data.profile;setCalibrationVisible(false);setTestsVisible(true);}}});
  showPage('imu');poll();setInterval(()=>{if(!document.hidden)poll();},50);
})();
