/* One new factory-ID servo per explicit start; existing numbered servos stay connected. */
(() => {
  'use strict';
  const IDS = [10,11,12,13,14,20,21,22,23,24,30,31,32,33,34];
  const PHASES = {preflight:0,numbering:1,centering:2,finishing:3};
  const START_TIMEOUT_MS = 1500, HEARTBEAT_LIMIT_MS = 1500;
  const validSession = value => typeof value === 'string' && /^[a-f0-9]{32}$/.test(value);
  function completedPrefix(value) {
    if(!Array.isArray(value)||value.length>IDS.length)return [];
    return value.every((id,index)=>id===IDS[index])?value.slice():[];
  }

  function visibilityMessage(event, parentWindow, origin) {
    return event.source === parentWindow && event.origin === origin &&
      event.data?.type === 'duck-numbering-visibility' && event.data.version === 1 &&
      typeof event.data.visible === 'boolean';
  }

  function phaseStates(state, online, observedStep) {
    const result = ['pending','pending','pending','pending'];
    if (!online || !state) return result;
    if (state.phase === 'complete' && state.completed === true && state.torque_off === true) return result.fill('done');
    const step = PHASES[state.phase];
    if (Number.isInteger(step)) {
      for(let i=0;i<step;i++)result[i]='done';
      result[step]='active';
    } else if (state.phase === 'error' || state.phase === 'stopped') {
      if(Number.isInteger(observedStep))result[observedStep]='error';
    }
    return result;
  }

  function createController(options) {
    const now = options.now || Date.now;
    const later = options.setTimeout || setTimeout, clearTimer = options.clearTimeout || clearTimeout;
    let visible = options.visible !== false, state = null, online = false, session = null;
    let ownedId = null, pending = null, stopping = false, uncertain = false, halted = false;
    let heartbeatBusy = false, stateBusy = false, portsBusy = false, lastHeartbeat = -Infinity;
    let generation = 0, observedStep = null, message = '', disposed = false;
    const notify = () => options.onChange?.(view());
    const clearChecks = () => options.clearChecks?.();
    const owns = () => validSession(session);

    function view() {
      const active = state?.active === true;
      const ready = visible && !disposed && online && !active && !pending && !stopping && !uncertain && !owns();
      let text = message;
      if (!text) {
        if (!online) text='状态待读取，暂不开始。';
        else if (active && !owns()) text='已有编号任务。本页只查看，不接管心跳。';
        else if (state?.phase==='preflight') text='扫描 ID 1 和已占用编号'+(Number.isInteger(state.scanned_ids)?' · '+state.scanned_ids+' / '+(state.scan_total||16):'');
        else if (state?.phase==='numbering') text='正在改为 ID '+state.target_id+'。';
        else if (state?.phase==='centering') text='正在归中 2047，保持无阻挡。';
        else if (state?.phase==='finishing') text='正在卸力、恢复参数并收尾。';
        else if (state?.phase==='complete') text=state.completed===true&&state.torque_off===true?'ID '+state.target_id+' 已卸力。保留已编号连接，断电新增一颗 ID 1 后再按。':'完成反馈尚未核对，先别换舵机。';
        else if (state?.phase==='full') text='15 个目标 ID 已占满；本次没有改号或运动。';
        else if (state?.phase==='error'||state?.phase==='stopped') text=state.message||'已停止；核对卸力反馈。';
        else text=visible?'保留已编号连接，只新增一颗 ID 1，再按自动编号。':'切回本页后再按。';
      }
      return {state,online,visible,owns:owns(),session,ready,pending:!!pending,stopping,uncertain,halted,
        canStop:owns()||!!pending,lockInputs:owns()||!!pending||stopping||active,
        message:text,phases:phaseStates(state,online,observedStep)};
    }

    function recordCompletion(result,id) {
      const completed=result?.active===false&&result.phase==='complete'&&result.completed===true&&result.torque_off===true&&result.target_id===id;
      if(completed){options.onCompleted?.(id);const next=result.next_id??IDS[IDS.indexOf(id)+1];if(IDS.includes(next))options.nextId?.(next);}
      return completed;
    }

    function adoptScannedTarget(result){
      if(owns()&&result?.retain_numbered===true&&result.target_selected===true&&IDS.includes(result.target_id))ownedId=result.target_id;
    }

    function finishOwned(result) {
      if(!owns())return false;
      adoptScannedTarget(result);
      const id=ownedId;
      session=null;ownedId=null;lastHeartbeat=-Infinity;generation+=1;clearChecks();
      return recordCompletion(result,id);
    }

    async function stopOwned(reason, keepalive=true) {
      const token=session,id=ownedId;
      session=null;ownedId=null;lastHeartbeat=-Infinity;generation+=1;halted=true;clearChecks();
      if(reason)message=reason;
      if(!validSession(token)){notify();return;}
      stopping=true;notify();
      try {
        const result=await options.request('numbering-stop',{session:token},{keepalive});
        if(result&&typeof result.active==='boolean'){state=result;online=true;recordCompletion(result,id);}
      } catch {online=false;message='卸力读回未确认，保持无遮挡并切断电源。';}
      finally {stopping=false;notify();}
    }

    function setVisible(value) {
      const next=Boolean(value)&&!disposed;
      if(next===visible)return;
      visible=next;generation+=1;
      if(!visible){
        if(pending)pending.cancelled=true;
        clearChecks();
        if(owns())void stopOwned('已离开编号页；已停止本页心跳并请求卸力。');
        else message=pending?'已离开编号页；迟到的开始回读会立即请求卸力。':'已离开编号页，切回后重新确认。';
      }
      notify();
    }

    async function readState() {
      if(stateBusy||disposed)return;
      stateBusy=true;const mark=generation;
      try {
        const result=await options.readState();
        if(mark!==generation||disposed)return;
        if(!result||typeof result.active!=='boolean'||typeof result.phase!=='string')throw Error('invalid_state');
        state=result;online=true;adoptScannedTarget(result);
        if(Number.isInteger(PHASES[result.phase]))observedStep=PHASES[result.phase];
        if(owns()&&result.active!==true){
          finishOwned(result);
        }
        if(!result.active&&!pending)uncertain=false;
        notify();
      } catch {
        if(mark!==generation||disposed)return;
        online=false;message='状态读取中断，暂不开始。';
        if(owns())await stopOwned('状态读取中断；已停止心跳并请求卸力。');
        notify();
      } finally {stateBusy=false;}
    }

    async function readPorts() {
      if(portsBusy||disposed)return;
      portsBusy=true;
      try {
        const result=await options.readPorts();
        if(!disposed)options.onPorts?.(Array.isArray(result?.ports)?result.ports:[]);
      } catch {if(!disposed){options.onPorts?.([]);message='串口未读到，点串口再试。';notify();}}
      finally {portsBusy=false;}
    }

    async function start(form) {
      if(!view().ready||!form||typeof form.port!=='string'||!form.port||!IDS.includes(form.next_id)||
        ['single','uninstalled','clear'].some(key=>form[key]!==true))return false;
      const attempt={cancelled:false,timedOut:false};pending=attempt;halted=false;message='正在请求连接，尚未认定完成。';
      generation+=1;observedStep=null;notify();
      const timer=later(()=>{
        if(pending!==attempt)return;
        attempt.cancelled=true;attempt.timedOut=true;uncertain=true;halted=true;clearChecks();
        message='开始请求超时，未续心跳。不要重复开始，等待读回。';notify();
      },START_TIMEOUT_MS);
      try {
        const result=await options.request('numbering-start',{
          port:form.port,next_id:form.next_id,single:true,uninstalled:true,clear:true,retain_numbered:true,session:null,permanent_pd:false
        });
        clearTimer(timer);pending=null;
        if(!validSession(result?.session))throw Error('unknown_start');
        session=result.session;ownedId=form.next_id;
        state=result;online=true;generation+=1;
        if(attempt.cancelled||!visible||disposed){
          await stopOwned(attempt.timedOut?'迟到的开始回读已收到；已请求卸力，未恢复心跳。':'已离开编号页；迟到的开始回读已请求卸力。');
          return false;
        }
        adoptScannedTarget(result);
        if(typeof result.active!=='boolean'||typeof result.phase!=='string'||result.target_id!==form.next_id&&result.retain_numbered!==true){await stopOwned('开始回读不一致，已请求卸力；不续心跳。');return false;}
        if(result.active!==true){finishOwned(result);notify();return false;}
        lastHeartbeat=now();message='';
        if(Number.isInteger(PHASES[result.phase]))observedStep=PHASES[result.phase];
        notify();return true;
      } catch {
        clearTimer(timer);if(pending===attempt)pending=null;
        session=null;ownedId=null;uncertain=true;halted=true;clearChecks();
        message='开始未确认，未续心跳；等待读回，勿重复开始。';notify();return false;
      }
    }

    async function stop() {
      if(pending){pending.cancelled=true;clearChecks();message='停止已记下；开始回读若迟到，会立即请求卸力。';notify();}
      if(owns())await stopOwned('停止已请求，等待卸力和收尾。');
    }

    async function heartbeat() {
      if(!owns()||!visible||disposed||halted)return;
      if(now()-lastHeartbeat>=HEARTBEAT_LIMIT_MS){await stopOwned('心跳超过时限；已请求卸力，不恢复授权。');return;}
      if(heartbeatBusy)return;
      heartbeatBusy=true;const token=session,mark=generation;
      try {
        const result=await options.request('numbering-heartbeat',{session:token});
        if(mark!==generation||session!==token||!visible||disposed)return;
        if(now()-lastHeartbeat>=HEARTBEAT_LIMIT_MS)throw Error('late_heartbeat');
        if(result?.active!==true){
          if(result&&typeof result.active==='boolean'){state=result;online=true;}
          finishOwned(result);notify();return;
        }
        adoptScannedTarget(result);
        if(result.target_id!==ownedId&&!(result.retain_numbered===true&&result.target_selected===false))throw Error('wrong_target');
        lastHeartbeat=now();state=result;online=true;message='';
        if(Number.isInteger(PHASES[result.phase]))observedStep=PHASES[result.phase];
        notify();
      } catch {
        if(mark===generation&&session===token)await stopOwned('心跳未确认；已停止续期并请求卸力。');
      } finally {heartbeatBusy=false;}
    }

    function dispose() {disposed=true;visible=false;generation+=1;if(pending)pending.cancelled=true;clearChecks();if(owns())void stopOwned('页面已关闭；已请求卸力。');}
    return {view,start,stop,heartbeat,readState,readPorts,setVisible,dispose};
  }

  const api={createController,visibilityMessage,phaseStates,completedPrefix,IDS,START_TIMEOUT_MS,HEARTBEAT_LIMIT_MS};
  if(typeof module==='object'&&module.exports){module.exports=api;return;}
  const get=id=>document.getElementById(id),embedded=new URLSearchParams(location.search).get('embed')==='1';
  let parentVisible=!embedded;
  if(embedded)document.body.classList.add('numbering-embedded');
  let completed=[];
  async function json(url,init,timeout=1200) {
    const controller=timeout?new AbortController():null;
    const timer=controller?setTimeout(()=>controller.abort(),timeout):null;
    try {
      const response=await fetch(url,{cache:'no-store',...init,...(controller?{signal:controller.signal}:{})});
      const value=await response.json();
      if(!response.ok)throw Error('request_failed');
      return value;
    } finally {if(timer!==null)clearTimeout(timer);}
  }
  function render(v) {
    if(v.state?.retain_numbered===true&&v.state.scan_complete===true&&Array.isArray(v.state.occupied_ids)){
      completed=IDS.filter(id=>v.state.occupied_ids.includes(id));
    }
    const busy=v.canStop||v.stopping||v.state?.active===true;
    const foreignComplete=v.state?.phase==='complete'&&!completed.includes(v.state.target_id);
    get('start').textContent=busy?'停止':completed.length===IDS.length?'重新扫描':'自动编号';get('start').dataset.action=busy?'stop':'start';
    get('start').disabled=busy?!v.canStop||!v.visible:!v.ready||!get('port').value;
    get('port').disabled=v.lockInputs;
    get('target-label').textContent=busy?'当前 ID':'下一颗 ID';
    get('target-id').textContent=String(busy&&v.state?.target_selected===false?'扫描中':busy&&IDS.includes(v.state?.target_id)?v.state.target_id:IDS.find(id=>!completed.includes(id))??'—');
    get('completed-count').textContent=completed.length+' / '+IDS.length;
    get('status').textContent=completed.length===IDS.length&&!busy?'15 个编号已占用；可重新扫描，不会覆盖已有编号。':foreignComplete?'保留已编号连接，只新增一颗 ID 1 后再按。':v.message;
    ['phase-connect','phase-numbering','phase-centering','phase-release'].forEach((id,i)=>get(id).dataset.state=foreignComplete?'pending':v.phases[i]);
    const position=v.state?.feedback?.position;
    get('feedback').textContent=v.online&&Number.isInteger(position)?'位置反馈 '+position+(v.state?.torque_off===true?' · 已读回卸力':' · 卸力待读回'):'';
  }
  const controller=createController({
    visible:parentVisible&&!document.hidden,onChange:render,
    onCompleted:id=>{if(IDS.includes(id)&&!completed.includes(id))completed.push(id);},
    readState:()=>json('/api/numbering'),readPorts:()=>json('/api/ports'),
    request:(op,payload,extra={})=>json('/api/action',{
      method:'POST',keepalive:extra.keepalive===true,headers:{'Content-Type':'application/json','X-Token':window.NUMBERING_TOKEN},
      body:JSON.stringify({op,...payload})
    },op==='numbering-start'?0:1000),
    onPorts:ports=>{
      const selected=get('port').value;get('port').replaceChildren();
      for(const port of ports)if(typeof port.port==='string')get('port').add(new Option(port.port+' · '+(port.name||''),port.port));
      if(!get('port').options.length)get('port').add(new Option('没有可用串口',''));
      else if(ports.some(port=>port.port===selected))get('port').value=selected;
      render(controller.view());
    }
  });
  get('port').addEventListener('focus',()=>{if(!controller.view().lockInputs)void controller.readPorts();});
  get('port').addEventListener('change',()=>render(controller.view()));
  get('start').addEventListener('click',event=>{
    if(event.isTrusted!==true)return;
    const state=controller.view();
    if(state.canStop){void controller.stop();return;}
    // A display hint only: the server rescans every press and picks a free ID.
    const target=IDS.find(id=>!completed.includes(id))??IDS[0];
    // Only this fresh, trusted button press confirms the permanently visible physical prerequisites.
    void controller.start({port:get('port').value,next_id:target,single:true,uninstalled:true,clear:true});
  });
  window.addEventListener('message',event=>{
    if(!embedded||!visibilityMessage(event,window.parent,location.origin))return;
    parentVisible=event.data.visible;
    controller.setVisible(parentVisible&&!document.hidden);
    if(parentVisible&&!document.hidden){void controller.readPorts();void controller.readState();}
  });
  document.addEventListener('visibilitychange',()=>{controller.setVisible(parentVisible&&!document.hidden);if(!document.hidden&&parentVisible)void controller.readState();});
  window.addEventListener('pagehide',()=>controller.dispose());
  setInterval(()=>{if(parentVisible&&!document.hidden){void controller.heartbeat();void controller.readState();}},500);
  render(controller.view());void controller.readPorts();void controller.readState();
  if(embedded&&window.parent!==window)window.parent.postMessage({type:'duck-numbering-ready',version:1},location.origin);
})();
