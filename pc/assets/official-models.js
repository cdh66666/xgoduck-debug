/* The approved model-library layout, bound to the deployed official model set. */
(() => {
 'use strict';const $=id=>document.getElementById(id);document.body.classList.add('embedded');
 const models=[
  {id:'walk',profile:'policy',name:'键盘控制',file:'xgoduck_walk.onnx',tag:'当前实机模型',note:'方向键 / WASD 行走，Shift 快走；与调试台使用同一份 Walk 权重。仿真可独立运行，实机稳定性仍待验收。'},
  {id:'getup',profile:'getup',name:'跌倒起身',file:'xgoduck_getup.onnx',note:'官方起身权重；直立后交还站立，任意跌姿恢复尚未通过。'},
  {id:'pick',profile:'pick',name:'空载 Pick',file:'xgoduck_pick.onnx',note:'历史空载低头、开合嘴正常；本轮未复测，叼取物体未验证。'},
  {id:'roulade',profile:'roulade',name:'翻滚模型',file:'xgoduck_roulade.onnx',note:'官方翻滚权重；交互仿真恢复不稳定，实机翻滚未通过。'}
 ];let selected=models[0],visible=true,basisKey='';
 $('model-count').textContent='4';
 $('page-message').textContent='选择模型，在本地物理仿真中试用。键盘与鼠标只控制虚拟小鸭子。';
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
  if(command==='run')$('official-model-preview').focus();
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
 function choose(model){if(selected!==model){previewState=null;previewControlPending=false;}selected=model;paintPreviewControls();$('model-name').textContent=model.name;$('runtime-message').textContent=model.note;$('preview-note').textContent=model.id==='walk'?'方向键 / WASD · 松键停步':'本地交互仿真 · 拖起 / 丢下 / 施力';
  preview(model);
  document.querySelectorAll('#model-list button').forEach(b=>{b.setAttribute('aria-selected',String(b.dataset.id===model.id));b.classList.toggle('selected',b.dataset.id===model.id);});
 }
 function drawRow(model){model.button.innerHTML='<strong>'+model.name+'</strong>'+(model.tag?'<span class="model-tag">'+model.tag+'</span>':'')+'<small>'+model.file+'</small>';model.button.title=model.sha?'权重 SHA256 '+model.sha:model.note;}
 for(const model of models){const button=document.createElement('button');model.button=button;button.className='model-row';button.dataset.id=model.id;drawRow(model);button.onclick=()=>choose(model);$('model-list').append(button);}
 function paintBasis(basis){
  const digest=basis?.walk_policy?.sha256;
  const key=/^[a-f0-9]{64}$/i.test(digest||'')?digest:'';
  if(key===basisKey)return;basisKey=key;
  const walk=models[0];walk.sha=key;
  drawRow(walk);
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
   $('runtime-state').textContent=`本地 MuJoCo · ${e.data.policy_enabled&&!e.data.paused?'仿真运行中':'仿真已暂停'}`;
   $('runtime-state').title='所选权重 SHA256 '+String(e.data.model_sha||'');
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
