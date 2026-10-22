/* The approved model-library layout, bound to the deployed official model set. */
(() => {
 'use strict';const $=id=>document.getElementById(id);document.body.classList.add('embedded');
 const models=[
  {id:'walk',profile:'policy',name:'站立 / 行走',file:'xgoduck_walk.onnx',note:'原版权重；本轮实机站立失稳、出现后撤，稳定性未通过。'},
  {id:'getup',profile:'getup',name:'跌倒起身',file:'xgoduck_getup.onnx',note:'官方起身权重；直立后交还站立，任意跌姿恢复尚未通过。'},
  {id:'pick',profile:'pick',name:'空载 Pick',file:'xgoduck_pick.onnx',note:'历史空载低头、开合嘴正常；本轮未复测，叼取物体未验证。'},
  {id:'roulade',profile:'roulade',name:'翻滚模型',file:'xgoduck_roulade.onnx',note:'官方翻滚权重；交互仿真恢复不稳定，实机翻滚未通过。'}
 ];const fast={id:'fast',profile:'policy',name:'R5 无EVA · 前进 / 快跑',file:'xgoduck_fast_forward.onnx',note:'W直线0.2档，Shift+W直线0.3档；实物稳定性未通过。'};
 const navigation={id:'navigation',profile:'policy',name:'原版模型 · 已验证后退 / 转向',file:'xgoduck_walk.onnx',note:'S后退指令0.3档，仿真实测约0.18–0.22 m/s；A/D原地转向1 rad/s。仅这些已测点，实物待验。'};let selected=models[0],visible=true,basisKey='';
 $('import-open').hidden=true;$('import-dialog').remove();$('model-count').textContent='4';
 $('page-message').textContent='在此运行所选模型的本地仿真；无需连接机器人。实机稳定性另行验证。';
 $('model-preview').innerHTML='<iframe id="official-model-preview" title="模型实时物理仿真" style="border:0;width:100%;height:100%"></iframe>';
 let previewState=null,previewControlPending=false;
 $('deploy').textContent='仿真运行此模型';$('deploy').disabled=true;
 $('stop').disabled=true;$('stop').textContent='暂停仿真';
 function paintPreviewControls(){
  const ready=previewState?.model_id==='official-'+selected.id&&/^[a-f0-9]{64}$/i.test(previewState?.model_sha||'');
  const running=ready&&previewState.policy_enabled&&!previewState.paused;
  $('deploy').disabled=!ready||previewControlPending||running;
  $('stop').disabled=!ready||previewControlPending||!running;
 }
 function controlPreview(command){
  if(previewControlPending||previewState?.model_id!=='official-'+selected.id||!/^[a-f0-9]{64}$/i.test(previewState?.model_sha||''))return;
  previewControlPending=true;paintPreviewControls();
  $('official-model-preview').contentWindow.postMessage({type:'duck-simulation-control',version:1,command,
   model_id:'official-'+selected.id,expected_sha:previewState.model_sha},location.origin);
 }
 $('stop').onclick=()=>controlPreview('pause');
 $('deploy').onclick=()=>controlPreview('run');
 let previewRequest=0,closeSerial=0,previewTask=Promise.resolve();const closures=new Map();
 function closePreview(){return new Promise((resolve,reject)=>{const requestId=++closeSerial,timer=setTimeout(()=>{closures.delete(requestId);reject(Error('旧仿真尚未关闭，请重新选择模型'));},20000);closures.set(requestId,()=>{clearTimeout(timer);resolve();});$('official-model-preview').contentWindow.postMessage({type:'duck-simulation-close',requestId},location.origin);});}
 function preview(model){
  const request=++previewRequest,source='/assets/model-training.html?model=official-'+model.id+'&controls=external';
  previewTask=previewTask.then(async()=>{
   if(request!==previewRequest)return;const frame=$('official-model-preview');if(frame.getAttribute('src')===source)return;
   if(frame.hasAttribute('src'))await closePreview();if(request!==previewRequest)return;frame.src=source;
  }).catch(error=>{$('runtime-state').textContent=error.message;});
 }
 function choose(model){if(selected!==model){previewState=null;previewControlPending=false;}selected=model;paintPreviewControls();$('model-name').textContent=model.name;$('runtime-message').textContent=model.note;$('preview-note').textContent='本地交互仿真 · 拖起 / 丢下 / 施力';
  preview(model);
  document.querySelectorAll('#model-list button').forEach(b=>{b.setAttribute('aria-selected',String(b.dataset.id===model.id));b.classList.toggle('selected',b.dataset.id===model.id);});
 }
 function drawRow(model){model.button.innerHTML='<strong>'+model.name+'</strong><small style="display:block;margin-top:8px">'+model.file+(model.sha?' · SHA '+model.sha.slice(0,12):'')+'</small>';}
 for(const model of models){const button=document.createElement('button');model.button=button;button.className='model-row';button.dataset.id=model.id;drawRow(model);button.onclick=()=>choose(model);$('model-list').append(button);}
 function paintBasis(basis){
  const basic=basis?.basic_keyboard_policy?.active===true;
  const policy=basis?.walk_policy,active=policy?.active===true&&/^[a-f0-9]{64}$/i.test(policy.sha256||'');
  const fastPolicy=basis?.fast_policy,fastActive=!basic&&fastPolicy?.active===true&&/^[a-f0-9]{64}$/i.test(fastPolicy.sha256||'');
  const navPolicy=basis?.navigation_policy,navActive=!basic&&navPolicy?.active===true&&/^[a-f0-9]{64}$/i.test(navPolicy.sha256||'');
  const key=(active?policy.sha256:'official')+'/'+(fastActive?fastPolicy.sha256:'no-fast')+'/'+(navActive?navPolicy.sha256:'no-navigation');if(key===basisKey)return;basisKey=key;
  if(fastActive){fast.sha=fastPolicy.sha256;if(!fast.button){const button=document.createElement('button');fast.button=button;button.className='model-row';button.dataset.id=fast.id;button.onclick=()=>choose(fast);$('model-list').append(button);}fast.button.hidden=false;drawRow(fast);}else if(fast.button){fast.button.hidden=true;if(selected===fast)choose(models[0]);}
  if(navActive){navigation.sha=navPolicy.sha256;if(!navigation.button){const button=document.createElement('button');navigation.button=button;button.className='model-row';button.dataset.id=navigation.id;button.onclick=()=>choose(navigation);$('model-list').append(button);}navigation.button.hidden=false;drawRow(navigation);}else if(navigation.button){navigation.button.hidden=true;if(selected===navigation)choose(models[0]);}
  $('model-count').textContent=String(4+Number(fastActive)+Number(navActive));
  const walk=models[0];walk.name=basic?'基础键盘 · 站立 / 前后左右':active?'R5无EVA·速度训练':'站立 / 行走';walk.sha=active?policy.sha256:'';
  walk.note=basic?`47500基础单一actor · SHA ${walk.sha.slice(0,12)}。W/S/A/D、Shift+W和松键零速共用同一模型；实物站立与走停未通过。`:active?`版本 ${policy.run_id||'当前训练'} · SHA ${walk.sha.slice(0,12)}。物理行走待测试。原官方零速度站立已有实物确认、行走稳定性未通过，均为旧权重历史结论。`:'原版权重；本轮实机站立失稳、出现后撤，稳定性未通过。';
  if(fastActive){walk.name='原版站立 / 后退 / 转向';walk.note='原版零速度站立已有实物确认；后退、转向另验。W直线前进和Shift快跑由单独的R5新模型负责。';}
  if(navActive){walk.name='原版站立 / 头部控制';walk.note='原版零速度站立已有实物确认；头部普通控制沿用原接口。W/S/A/D使用单独训练的前进和导航模型，实物另验。';}
  drawRow(walk);$('page-message').textContent=fastActive?'W直行和Shift+W快跑使用R5无EVA模型；后退及转向保留原模型。实物行走与翻滚待验。':active?'当前为官方运行链 + R5无EVA速度训练行走权重；其他三个模型保留原版本。':'当前基于固定官方运行链；模型来源与实物结论分别记录。';
  if(navActive)$('page-message').textContent='W / Shift+W 使用新前进模型；S / A / D 复用已验证原模型档位。原版站立、起身、Pick和翻滚保留，实物另验。';
  if(selected===walk)choose(walk);
 }
 let latest=null;
 function sendVisibility(){ $('official-model-preview')?.contentWindow?.postMessage({type:'duck-model-preview-visibility',visible:visible&&!document.hidden},location.origin); }
 function consume(s){latest=s;if(s.baseline)paintBasis(s.baseline);if(!previewState)$('runtime-state').textContent='本地 MuJoCo · 等待所选 ONNX';}
 window.addEventListener('message',e=>{
  if(e.origin!==location.origin)return;
  if(e.source===window.parent){
   if(e.data?.type==='bench-models-visibility'){visible=e.data.visible===true;sendVisibility();if(visible&&latest)consume(latest);}
   if(e.data?.type==='bench-models-status')consume(e.data);
  }
  if(e.source===$('official-model-preview')?.contentWindow&&e.data?.type==='duck-simulation-closed'){const done=closures.get(e.data.requestId);if(done){closures.delete(e.data.requestId);done();}}
  if(e.source===$('official-model-preview')?.contentWindow&&e.data?.type==='duck-simulation-preview-state'&&e.data.model_id==='official-'+selected.id){
   previewState=e.data;paintPreviewControls();selected.sha=e.data.model_sha;drawRow(selected);
   $('runtime-state').textContent=`本地 MuJoCo · ${e.data.policy_enabled&&!e.data.paused?'仿真运行中':'仿真已暂停'} · SHA ${String(e.data.model_sha||'').slice(0,12)}`;
  }
  if(e.source===$('official-model-preview')?.contentWindow&&e.data?.type==='duck-simulation-control-result'&&e.data.model_id==='official-'+selected.id){
   previewControlPending=false;paintPreviewControls();if(e.data.ok!==true)$('runtime-state').textContent=e.data.error||'仿真操作未确认';
  }
 });
 $('official-model-preview').addEventListener('load',sendVisibility);
 document.addEventListener('visibilitychange',sendVisibility);
 choose(selected);
 if(window.parent!==window)window.parent.postMessage({type:'bench-models-ready'},location.origin);
})();
