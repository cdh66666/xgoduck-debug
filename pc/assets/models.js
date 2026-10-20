(function () {
  'use strict';
  const activeStates = new Set(['running', 'active', 'deployed', 'starting', 'deploying', 'loading', 'arming', 'checking', 'stopping']);
  const stateLabels = {idle:'尚未运行模型',unloaded:'尚未加载模型',ready:'模型预检完成，尚未启动',stopped:'模型已停止',running:'鸭子本地运行中',active:'模型运行中',deployed:'模型已部署',starting:'正在启动模型',deploying:'正在部署模型',loading:'正在加载模型',arming:'按当前位置接管模型',checking:'正在核对运行状态',stopping:'正在停止 / 卸力',blocked:'暂不能启动模型',error:'模型运行异常',fault:'模型运行故障',stop_unconfirmed:'停止 / 卸力尚未确认',failed:'部署失败',unknown:'正在读取模型运行时',offline:'主控离线',unavailable:'主控模型运行时未安装'};
  function safePreview(raw, base) {
    try { const url = new URL(raw, base); return url.origin === new URL(base).origin && ['http:', 'https:'].includes(url.protocol) ? url.href : ''; } catch (_) { return ''; }
  }
  function previewKind(model) {
    const kind = String(model.preview_kind || '').toLowerCase();
    if (['video','image','iframe'].includes(kind)) return kind;
    const path = String(model.preview_url || '').split('?')[0];
    if (/\.(mp4|webm)$/i.test(path) || ['mp4','webm'].includes(kind)) return 'video';
    if (/\.(gif|webp|png|jpg|jpeg)$/i.test(path) || ['gif','webp'].includes(kind)) return 'image';
    if (['trajectory','replay'].includes(kind)) return 'iframe';
    return '';
  }
  function trainingFacts(training) {
    if (!training || typeof training !== 'object') return [];
    const labels = {version:'版本',algorithm:'算法',task:'任务',steps:'训练步数',timesteps:'训练步数',iterations:'迭代次数',reward:'奖励',mean_reward:'平均奖励',duration:'时长',duration_seconds:'时长 / 秒',trained_at:'训练时间',environment:'复验环境',episodes:'历史评估回合',simulation_successes:'历史评估通过'};
    return Object.entries(training).filter(([key,value]) => labels[key] && ['number','string'].includes(typeof value) && String(value).trim()).slice(0,6).map(([key,value]) => [labels[key],String(value)]);
  }
  function runtimeView(runtime, selected, busy, connected) {
    const state = String(runtime?.state || 'idle').toLowerCase();
    const active = activeStates.has(state);
    const matching = !!selected && String(runtime?.active_model_id || runtime?.model_id || '') === String(selected.id);
    const ready = selected?.compatibility?.ready === true;
    const error = ['error','failed','blocked','fault','stop_unconfirmed'].includes(state);
    return {state, active, matching, label:stateLabels[state] || '状态待确认', canDeploy:connected && !busy && !!selected && ready && ['idle','unloaded','ready','stopped','error','failed','blocked','fault'].includes(state), canStop:connected && busy !== 'stop' && (active || error || !!runtime?.motion_owner), error};
  }
  if (typeof module !== 'undefined' && module.exports) module.exports = {safePreview,previewKind,trainingFacts,runtimeView};
  if (typeof document === 'undefined') return;
  const $ = id => document.getElementById(id);
  const ui = Object.fromEntries(['page-message','model-count','model-list','library-empty','model-name','model-preview','preview-note','runtime-dot','runtime-state','runtime-message','deploy','stop','import-dialog','import-form','import-message','import-submit','import-model','import-name','import-preview'].map(id=>[id,$(id)]));
  const data = {models:[],runtime:{state:'idle'},selected:null,busy:false,stopping:false,connected:false,previewKey:'',listKey:'',refreshing:false,timer:null};
  let token = window.COMMISSION_TOKEN;
  if (new URLSearchParams(location.search).get('embed') === '1') document.body.classList.add('embedded');
  function displayMessage(text) {const value=String(text || '');const labels={unloaded:'选择模型后，可部署到鸭子本地独立运行。',ready:'板端推理预检通过；点击部署后开始运行。',loading:'正在加载所选模型。',arming:'按当前关节位置接管，不先回正。',running:'鸭子本地独立运行，关闭网页不影响。',stopped:'已停止模型。'};return /Fresh pinned recovery evidence required/i.test(value) ? '主控暂未连接，正在重试' : (labels[value] || value);}
  function message(text, error = false) { ui['page-message'].textContent = displayMessage(text); ui['page-message'].classList.toggle('error',error);ui['page-message'].hidden=!error; }
  async function request(url, options = {}, retried = false) {
    const controller = new AbortController();
    const timeout = setTimeout(()=>controller.abort(), options.body instanceof FormData ? 120000 : 20000);
    try {
      const response = await fetch(url,{...options,cache:'no-store',signal:controller.signal,headers:{'X-Token':token || '',...options.headers}});
      if ((response.status === 401 || response.status === 403) && !retried) {
        const session = await fetch('/api/local-session',{cache:'no-store',signal:controller.signal});
        if (session.ok) { token = (await session.json()).token; return request(url,options,true); }
      }
      const content = await response.json().catch(()=>({}));
      if (!response.ok || content.ok === false) throw new Error(content.error?.message || content.error?.reason || content.error || content.message || `请求失败（${response.status}）`);
      return content;
    } catch (error) { if (error.name === 'AbortError') throw new Error('请求超时，请检查调试服务后重试。'); throw error; }
    finally { clearTimeout(timeout); }
  }
  const current = () => data.models.find(model=>String(model.id) === String(data.selected));
  function node(tag, text, className) { const result = document.createElement(tag); if(text !== undefined) result.textContent = text; if(className) result.className = className; return result; }
  function renderList() {
    const key=JSON.stringify([data.selected,data.runtime.state,data.runtime.active_model_id || data.runtime.model_id,data.models.map(model=>[model.id,model.name,model.compatibility?.ready])]);
    if(key===data.listKey)return;data.listKey=key;
    const focused=document.activeElement?.dataset?.modelId;
    const fragment = document.createDocumentFragment();
    data.models.forEach(model=>{
      const button = node('button',undefined,'model-row'); button.type='button'; button.setAttribute('aria-current',String(String(model.id)===String(data.selected)));
      button.dataset.modelId=String(model.id);
      button.append(node('strong',model.name || '未命名模型'));
      const activeId = data.runtime.active_model_id || data.runtime.model_id;
      const running = activeStates.has(String(data.runtime.state).toLowerCase()) && String(activeId) === String(model.id);
      if(running||model.compatibility?.ready!==true)button.append(node('small',running?'正在使用':'等待适配',running?'running-tag':''));
      button.addEventListener('click',()=>{data.selected=model.id;render();}); fragment.append(button);
    });
    ui['model-list'].replaceChildren(fragment); ui['model-count'].textContent=String(data.models.length); ui['library-empty'].hidden=!!data.models.length;
    if(focused)Array.from(ui['model-list'].children).find(button=>button.dataset.modelId===focused)?.focus();
  }
  function clearMedia() { const video = ui['model-preview'].querySelector('video'); if(video) video.pause(); ui['model-preview'].replaceChildren(); }
  function renderPreview(model) {
    const key = model ? `${model.id}|${model.preview_url || ''}|${model.preview_kind || ''}` : '';
    if(key === data.previewKey && ui['model-preview'].childNodes.length) return;
    data.previewKey=key;clearMedia();
    const simulation=model?.compatibility?.ready===true;
    const url = simulation ? `/assets/model-training.html?model=${encodeURIComponent(model.id)}` : model?.preview_url && safePreview(model.preview_url,location.href);
    const kind = simulation ? 'iframe' : model && previewKind(model);
    if(!url || !kind) { const empty=node('div',undefined,'preview-empty'); empty.append(node('p',model ? '这个模型还没有训练成果预览' : '导入模型，查看对应的动态成果')); ui['model-preview'].append(empty); return; }
    const media = node(kind === 'image' ? 'img' : kind === 'video' ? 'video' : 'iframe');
    if(kind==='image') {media.alt=`${model.name || '模型'}的训练成果动态图`;media.loading='lazy';}
    if(kind==='video') {media.controls=true;media.loop=true;media.muted=true;media.playsInline=true;media.preload='metadata';}
    if(kind==='iframe') {media.title=`${model.name || '模型'}交互仿真`;media.setAttribute('sandbox','allow-scripts allow-same-origin');media.loading='lazy';}
    media.addEventListener('error',()=>{if(data.previewKey!==key)return;clearMedia();ui['model-preview'].append(node('p','预览加载失败，请重新导入对应的动态图或视频。','preview-empty'));});
    media.src=url;ui['model-preview'].append(media);
    ui['preview-note'].textContent=simulation?'实时物理仿真，不驱动实物':'已保存的训练资料';
  }
  function render() {
    const model=current();renderList();renderPreview(model);
    ui['model-name'].textContent=model?.name || '选择一个模型';
    const view=runtimeView(data.runtime,model,data.stopping ? 'stop' : data.busy,data.connected);
    ui.deploy.disabled=!view.canDeploy;ui.stop.disabled=!view.canStop;
    ui.deploy.textContent=data.busy==='deploy' ? '正在部署…' : (view.active && view.matching ? '正在使用' : '部署到小鸭子');
    ui.stop.textContent=data.stopping ? '正在停止…' : '停止 / 卸力';
    ui['runtime-state'].textContent=data.connected ? view.label+((view.state==='running'&&Number.isFinite(data.runtime.accepted_frame))?' · 主控帧 '+data.runtime.accepted_frame:'') : '服务连接中断';
    ui['runtime-dot'].classList.toggle('active',view.active && data.connected);ui['runtime-dot'].classList.toggle('error',view.error || !data.connected);
    const reason=model && model.compatibility?.ready !== true ? (model.compatibility?.reason || '模型尚未适配，暂不能部署。') : '';
    ui['runtime-message'].hidden=!view.error;
    ui['runtime-message'].textContent=!data.connected ? '恢复连接后会重新读取实际状态。' : displayMessage(reason || data.runtime.message || (model ? '部署后将在小鸭子上运行这个模型。' : '选择模型后可查看部署条件。'));
  }
  async function refresh(quiet = false) {
    if(data.refreshing) return;data.refreshing=true;
    try {
      const result=await request('/api/models');
      if(!Array.isArray(result.models) || !result.runtime || typeof result.runtime.state!=='string')throw new Error('模型服务返回的状态不完整，请重试。');
      data.models=Array.isArray(result.models)?result.models:[];data.runtime=result.runtime || {state:'idle'};data.connected=true;
      if(!current()) data.selected=data.models[0]?.id || null;
      if(!quiet) message(data.models.length ? '选择模型，直接交互仿真；部署后鸭子本地独立运行。' : '导入模型文件和对应的训练成果，开始建立模型库。');
      render();
    } catch(error) {data.connected=false;message(error.message,true);render();}
    finally {data.refreshing=false;}
  }
  async function action(kind) {
    if((data.busy && kind!=='stop') || (kind==='stop' && data.stopping)) return;const model=current();if(kind==='deploy' && !model) return;
    if(kind==='stop')data.stopping=true;else data.busy=kind;
    message(kind==='deploy'?'正在部署，请稍候…':'正在停止模型并卸力…');render();
    try {
      const result=await request(kind==='deploy'?`/api/models/${encodeURIComponent(model.id)}/deploy`:'/api/models/stop',{method:'POST',headers:{'Content-Type':'application/json'},body:'{}'});
      if(result.runtime) data.runtime=result.runtime;
      await refresh(true);
      if(!data.connected)return;
      const state=String(data.runtime.state).toLowerCase();
      const failed=['error','failed','blocked','fault','stop_unconfirmed'].includes(state);
      message(failed ? (data.runtime.message || '操作失败，请检查模型状态。') : (result.message || (kind==='deploy' ? '部署请求已处理，以下方实际运行状态为准。' : '停止请求已处理，以下方实际状态为准。')),failed);
    } catch(error) {message(error.message,true);await refresh(true);}
    finally {if(kind==='stop')data.stopping=false;else data.busy=false;render();}
  }
  function closeImport() {if(data.busy==='import')return;ui['import-dialog'].close();}
  $('import-open').addEventListener('click',()=>{ui['import-message'].textContent='';ui['import-message'].classList.remove('error');ui['import-dialog'].showModal();});
  $('import-close').addEventListener('click',closeImport);$('import-cancel').addEventListener('click',closeImport);
  ui['import-dialog'].addEventListener('cancel',event=>{if(data.busy==='import')event.preventDefault();});
  ui['import-model'].addEventListener('change',()=>{if(!ui['import-name'].value)ui['import-name'].value=ui['import-model'].files[0]?.name.replace(/\.onnx$/i,'') || '';});
  ui['import-form'].addEventListener('submit',async event=>{
    event.preventDefault();if(data.busy)return;
    if(!ui['import-form'].reportValidity())return;
    const file=ui['import-model'].files[0];const preview=ui['import-preview'].files[0];
    if(!file || !/\.onnx$/i.test(file.name)){ui['import-message'].textContent='请选择 ONNX 模型文件。';ui['import-message'].classList.add('error');return;}
    if(preview && !/\.(gif|webp|mp4|webm)$/i.test(preview.name)){ui['import-message'].textContent='训练成果支持 GIF、WebP、MP4 或 WebM。';ui['import-message'].classList.add('error');return;}
    data.busy='import';ui['import-submit'].disabled=true;ui['import-submit'].textContent='正在保存…';ui['import-message'].textContent='正在上传文件，请保持页面打开。';ui['import-message'].classList.remove('error');render();
    try {
      const form=new FormData(ui['import-form']);if(!preview)form.delete('preview');
      const result=await request('/api/models/import',{method:'POST',body:form});
      const id=result.model?.id || result.model_id || result.id;if(id)data.selected=id;
      await refresh(true);ui['import-form'].reset();ui['import-dialog'].close();message('模型已保存，可查看训练成果和部署条件。');
    } catch(error) {ui['import-message'].textContent=error.message;ui['import-message'].classList.add('error');}
    finally {data.busy=false;ui['import-submit'].disabled=false;ui['import-submit'].textContent='保存模型';render();}
  });
  ui.deploy.addEventListener('click',()=>action('deploy'));ui.stop.addEventListener('click',()=>action('stop'));
  let pageVisible=!document.hidden;
  function previewVisibility(visible){
    const video=ui['model-preview'].querySelector('video');if(!visible&&video)video.pause();
    const frame=ui['model-preview'].querySelector('iframe');
    if(frame?.contentWindow)frame.contentWindow.postMessage({type:'duck-model-preview-visibility',visible},location.origin);
  }
  window.addEventListener('message',event=>{
    if(event.origin!==location.origin||event.source!==parent||event.data?.type!=='duck-model-library-visibility'||event.data.version!==1)return;
    pageVisible=event.data.visible===true;previewVisibility(pageVisible&&!document.hidden);
    if(pageVisible&&!document.hidden)refresh(true);
  });
  document.addEventListener('visibilitychange',()=>{previewVisibility(pageVisible&&!document.hidden);if(pageVisible&&!document.hidden)refresh(true);});
  window.addEventListener('pagehide',()=>{clearInterval(data.timer);const video=ui['model-preview'].querySelector('video');if(video)video.pause();});
  refresh();data.timer=setInterval(()=>{if(pageVisible&&!document.hidden && !data.busy)refresh(true);},3000);
}());
