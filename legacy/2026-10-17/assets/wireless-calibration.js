/* Wireless calibration uses the fixed local proxy. Selection and polling are read-only. */
(function (scope) {
  'use strict';
  const IDS = [10,11,12,13,14,20,21,22,23,24,30,31,32,33,34];
  const SCHEMA = 'wireless-calibration-v2';
  const GROUPS = Object.freeze({left:Object.freeze(IDS.slice(0,5)),right:Object.freeze(IDS.slice(5,10)),head:Object.freeze(IDS.slice(10))});
  const GROUP_LABELS = Object.freeze({left:'左腿',right:'右腿',head:'头部'});
  const groupForId = id => Object.keys(GROUPS).find(group=>GROUPS[group].includes(id));
  const STATUS_URL = '/api/wireless-calibration';
  const ACTION_URL = '/api/wireless-calibration/action';
  const LOCAL_SESSION_URL = '/api/local-session';
  const newRequestOps = new Set(['off_all','capture_group','capture_all','arm','home_prepare','cancel','home_stop','sequence_begin','sequence_stop']);
  const localUnsent = value => ['local_session_expired','local_session_unavailable','local_request_cancelled'].includes(errorReason(value));
  function createRequest(options) {
    return async (op,payload,extra={}) => {
      // New user intent (and stopping) gets the current PC token before its
      // first POST. Never replay a POST or refresh a running motion's lease.
      if(newRequestOps.has(op)){
        try{
          const response=await options.fetch(LOCAL_SESSION_URL,{cache:'no-store',signal:AbortSignal.timeout(3000)});
          const session=await response.json();
          if(!response.ok||typeof session.token!=='string'||!/^[0-9a-f]{48}$/.test(session.token))throw Error();
          options.setToken(session.token);
        }catch{
          const result={code:'local_session_unavailable'};
          throw Object.assign(new Error(formatError(result)),{result});
        }
      }
      if(extra.guard&&!extra.guard()){
        const result={code:'local_request_cancelled'};
        throw Object.assign(new Error(formatError(result)),{result});
      }
      const response=await options.fetch(ACTION_URL,{method:'POST',keepalive:extra.keepalive===true,
        headers:{'Content-Type':'application/json','X-Token':options.getToken()},body:JSON.stringify({op,...payload})});
      const result=await response.json();
      if(!response.ok)throw Object.assign(new Error(formatError(result)),{result});
      return result;
    };
  }
  const uint32 = value => Number.isInteger(value) && value >= 0 && value <= 4294967295;
  const encoder = value => Number.isInteger(value) && value >= 0 && value <= 4095;
  const nativeEncoder = value => Number.isInteger(value) && value >= -32767 && value <= 32767;
  const NATIVE_VERSION='2026-10-13-wireless-calibration-v4';
  const PREVIOUS_HOME_VERSION='2026-10-13-wireless-calibration-v5';
  const LEGACY_HOME_VERSION='2026-10-13-wireless-calibration-v6';
  const HOME_VERSION='2026-10-15-wireless-calibration-v7';
  const NATIVE_DOMAIN='native-sign-magnitude-v1';
  const nativeContract = s => [NATIVE_VERSION,PREVIOUS_HOME_VERSION,LEGACY_HOME_VERSION,HOME_VERSION].includes(s?.version);
  function recordValid(record) {
    if(!record||!nativeEncoder(record.position))return false;
    if(record.coordinate_domain===NATIVE_DOMAIN)return record.mode===4&&record.signed_steps===record.position&&
      Number.isInteger(record.raw_word)&&record.raw_word===(record.position<0?0x8000-record.position:record.position);
    if(record.coordinate_domain==='single-turn-0-4095')return record.mode===0&&encoder(record.position)&&record.raw_word===record.position&&record.signed_steps===record.position;
    if(record.coordinate_domain!==undefined)return false;
    return encoder(record.position)&&(record.mode===undefined||record.mode===0||record.mode===4)&&
      (record.raw_word===undefined||record.raw_word===record.position)&&(record.signed_steps===undefined||record.signed_steps===record.position);
  }
  const groupRecordsSaved=(records,group)=>Object.hasOwn(GROUPS,group)&&GROUPS[group].every(id=>recordValid(records?.[id]));
  const motionSafe = (row,s) => row?.valid===true&&encoder(row.position)&&
    (row.motion_valid===true||row.motion_valid===undefined&&!nativeContract(s)&&row.coordinate_domain===undefined);
  const observedPosition = row => nativeEncoder(row?.signed_steps)?row.signed_steps:nativeEncoder(row?.position)?row.position:null;
  const receiptFile = s => nativeContract(s)||s?.native_only===true?s?.native_file:s?.file;
  const receiptContext = s => !nativeContract(s)||(s.native_only===true&&s.model_deployment_ready===false);
  const sameRecord = (a,b) => ['position','mode','raw_word','signed_steps','coordinate_domain'].every(key=>a?.[key]===b?.[key]);
  const nativeFileVerified = s => !nativeContract(s)||(receiptFile(s)?.path==='data/zero_pos.native.json'&&
    typeof s.group_file?.sha256==='string'&&/^[0-9a-f]{64}$/.test(s.group_file.sha256));
  function jointReadout(row,fresh) {
    if(!fresh||row?.fresh!==true)return 'ID '+(row?.id??'—')+'：等待新鲜位置';
    const value=observedPosition(row);if(value===null)return 'ID '+row.id+'在线，位置编码未返回';
    const prefix='ID '+row.id+'在线，编码 '+value;
    if(row.raw_word===0x8000)return prefix+'：负零编码无效，暂不记录';
    if(row.valid!==true){
      if(!encoder(value)&&row.coordinate_domain!==NATIVE_DOMAIN)return prefix+'，超出旧单圈校验';
      if(row.mode!==undefined&&![0,4].includes(row.mode))return prefix+'：模式 '+row.mode+' 不支持记录';
      return prefix+'：'+({position_stale:'位置过期',metadata_stale:'模式/卸力数据过期',invalid_position:'编码域待核对'}[row.blocked_reason]||'编码域待核对');
    }
    return prefix+(motionSafe(row)?'':' · 原生值可记录，小步仍受单圈界限限制');
  }
  const hash64=value=>typeof value==='string'&&/^[0-9a-f]{64}$/.test(value);
  const HOME_CONTRACT_REVISION=5, HOME_MOTION_LIMIT_MS=450000;
  function homePositionAllowed(id,position,target){
    if(!nativeEncoder(position)||!nativeEncoder(target))return false;
    const q=-(position-target)*360;
    if(id===30)return position-target>=-682&&position-target<=2047;
    if(id===31)return q>=-105*4096&&q<=105*4096;
    return Math.abs(position-target)<=1024;
  }
  const homeUnreleased=h=>h?.active===true||h?.session_active===true;
  const completeHomeHold=h=>h?.contract_revision===HOME_CONTRACT_REVISION&&h.tolerance_ticks===57&&h.phase==='holding'&&
    h.arrival_verified===true&&h.active===true&&h.progress?.active_id===0&&h.progress.stage===9&&
    h.progress.arrived_mask===0x7fff&&h.progress.enabled_mask===0x7fff&&
    h.progress.primed_mask===0x7fff&&h.progress.fresh_mask===0x7fff;
  const homeProgressNote=h=>{
    const id=h?.progress?.active_id,index=IDS.indexOf(id);
    return index<0?'逐颗慢速回中；可随时停止':'第'+(index+1)+'/15颗 · ID'+id+' · '+(h.progress.stage===8?'稳定到位':'正在回中');
  };
  function homeTargetMatches(p,id,index,record,row) {
    const target=p?.targets?.[index];
    if(!nativeEncoder(target)||!recordValid(record))return false;
    if(target===record.position)return true;
    const offset=p?.turn_offsets?.[index],verified=p?.turn_equivalence_verified_ids;
    const reference=p?.turn_reference;
    return p?.target_resolution==='audited-finite-turn-v3'&&
      hash64(reference?.sha256)&&reference.boot_id===p.boot_id&&reference.native_sha256===p.native_sha256&&
      typeof reference.app_instance_id==='string'&&/^[0-9a-f]{32}$/.test(reference.app_instance_id)&&
      Array.isArray(p.saved_targets)&&p.saved_targets.length===15&&p.saved_targets[index]===record.position&&
      Array.isArray(p.turn_offsets)&&p.turn_offsets.length===15&&[-1,1].includes(offset)&&
      Array.isArray(verified)&&new Set(verified).size===verified.length&&verified.every(value=>[10,22,30].includes(value))&&verified.includes(id)&&
      record.mode===4&&row?.mode===4&&p.modes?.[index]===4&&
      target===record.position+4096*offset&&!homePositionAllowed(id,row.position,record.position)&&homePositionAllowed(id,row.position,target);
  }
  function homeBlockedNote(v){
    if(!v.fresh)return '等待新鲜位置';
    if(v.s?.home?.tolerance_ticks!==57)return '归位程序待更新（5°容差）';
    const stale=IDS.find(id=>v.rows.get(id)?.fresh!==true);
    if(stale!==undefined)return 'ID '+stale+'位置过期，等待刷新';
    const staleMeta=IDS.find(id=>{const age=v.rows.get(id)?.metadata_age_ms;return !Number.isFinite(age)||age<0||age>750;});
    if(staleMeta!==undefined)return 'ID '+staleMeta+'模式/卸力数据过期';
    if(!v.allOff)return '托稳后点卸力';
    const records=v.records||v.s?.records;
    if(v.s?.complete!==true||v.s?.native_file?.verified!==true)return '完整零位文件待核对';
    const mismatch=IDS.find((id,index)=>{const mode=records?.[id]?.mode??v.s?.home?.plan?.modes?.[index];return [0,4].includes(mode)&&v.rows.get(id)?.mode!==mode;});
    if(mismatch!==undefined)return 'ID '+mismatch+'模式与零位不符';
    const invalid=IDS.find(id=>!v.valid(v.rows.get(id)));
    if(invalid!==undefined)return 'ID '+invalid+'位置编码待核对';
    const far=IDS.map((id,index)=>{const row=v.rows.get(id),record=records?.[id],p=v.s?.home?.plan;
      const target=homeTargetMatches(p,id,index,record,row)?p.targets[index]:record?.position;
      return {id,span:Math.abs((target??NaN)-(row?.position??NaN)),allowed:homePositionAllowed(id,row?.position,target)};})
      .filter(row=>Number.isFinite(row.span)&&!row.allowed).sort((a,b)=>b.span-a.span)[0];
    if(far&&v.rows.get(far.id)?.mode===4&&Math.abs(far.span-4096)<=1024)
      return 'ID '+far.id+' 回中坐标未通过校验；不要绕整圈调整';
    return far?'ID '+far.id+'超出该关节允许的回中角度，保持卸力':'等待归位数据刷新';
  }
  function homePlanReady(check) {
    const s=check.s,h=s?.home,p=h?.plan;
    if(!check.fresh||!check.policyLocked||!check.allOff||!check.idle||s?.torque_may_be_on!==false||s.mcu_state!==1||s?.version!==HOME_VERSION||h?.contract_revision!==HOME_CONTRACT_REVISION||h?.tolerance_ticks!==57||h?.supported!==true||h.ready!==true||h.active!==false||
      s.complete!==true||s.native_file?.verified!==true||s.native_file.path!=='data/zero_pos.native.json'||!hash64(s.native_file.sha256)||
      !p||p.range_profile!=='native-head-recovery-v2'||p.boot_id!==s.boot_id||p.native_sha256!==s.native_file.sha256||!Number.isInteger(p.max_span_ticks)||p.max_span_ticks<0||p.max_span_ticks>2047||
      !Array.isArray(p.targets)||p.targets.length!==15||!Array.isArray(p.modes)||p.modes.length!==15)return false;
    return IDS.every((id,index)=>{const row=check.rows.get(id),r=s.records?.[id],mode=p.modes[index],target=p.targets[index];
      return check.valid(row)&&recordValid(r)&&[0,4].includes(mode)&&row.mode===mode&&(r.mode===undefined||r.mode===mode)&&
        nativeEncoder(target)&&homeTargetMatches(p,id,index,r,row)&&(mode===4||encoder(target))&&homePositionAllowed(id,row.position,target);});
  }
  function homeReleased(check) {
    return check.fresh&&check.policyLocked&&check.allOff&&check.s?.torque_may_be_on===false&&check.s.mcu_state===1&&check.s?.home?.supported===true&&check.s.home.release_verified===true&&check.s.home.active===false&&
      check.s.armed?.active===false&&!check.s.session_active;
  }
  function homeIdleNote(v) {
    const h=v.s?.home;
    if(!v.fresh)return '等待新鲜反馈';
    if(homeReleased(v)&&h.tolerance_ticks===57&&h.arrival_verified===true)return '已归位、已卸力（5°容差）。';
    if(h?.phase==='fault'){
      if(!homeReleased(v))return '归位停止待确认；保持托稳。';
      return v.homeReady?'已卸力；上次回正失败，先查首故障':homeBlockedNote(v);
    }
    return v.homeReady?'已卸力，可回正 · 容差5°':homeBlockedNote(v);
  }
  function homeFaultNote(status) {
    const home=status?.home;
    if(!home||home.arrival_verified===true||['preparing','prepared','enabling','moving','holding'].includes(home.phase))return '';
    const faults=(Array.isArray(home.fault_history)?home.fault_history:[]).filter(fault=>
      fault.boot_id===status.boot_id&&fault.native_sha256===home.native_sha256&&
      Number.isInteger(fault.first_observed_error)&&fault.first_observed_error>0);
    const fault=[...faults].reverse().find(row=>['prepare','run'].includes(row.stage))||faults.at(-1);
    if(!fault)return '';
    const stages={read_mode:'模式核对',read_torque:'卸力核对',read_position:'起点核对',prime_echo:'起点写回',
      enable_readback:'上力核对',prime_settle:'原位稳定',ramp:'回中',final_settle:'到位稳定',hold:'保持',release:'卸力'};
    const errors={2:'反馈过期',3:'授权失效',6:'反馈错误',7:'模式错误',8:'位置错误',9:'电压保护',10:'温度保护',11:'范围保护',12:'运动超时',13:'卸力未确认'};
    const index=IDS.indexOf(fault.active_id),parts=[];
    if(index>=0)parts.push('ID '+fault.active_id);
    if(stages[fault.execution_stage])parts.push(stages[fault.execution_stage]);
    parts.push('MCU '+fault.first_observed_error+(errors[fault.first_observed_error]?' '+errors[fault.first_observed_error]:''));
    const feedback=fault.first_feedback,goal=feedback?.goals?.[index],position=feedback?.positions?.[index];
    const measured=index>=0&&Number.isInteger(feedback?.fresh_mask)&&(feedback.fresh_mask&(1<<index))!==0&&nativeEncoder(goal)&&nativeEncoder(position);
    return '上次首个故障：'+parts.join(' · ')+(measured?'；当时目标 '+goal+' / 实测 '+position+'（差 '+(goal-position)+' 步）':'');
  }

  function assess(envelope, now, receivedAt, selectedId, group = 'left') {
    const s = envelope?.status;
    const connected = envelope?.connection?.online === true;
    const interlocked = envelope?.connection?.update_interlocked === true;
    const supported = s?.schema === SCHEMA && s.supported === true && s.snapshot_version===3 && s.mcu_contract_revision===3;
    const fresh = connected && supported && s.fresh === true && uint32(s.boot_id) && uint32(s.sequence) &&
      Number.isFinite(s.age_ms) && s.age_ms >= 0 && s.age_ms < 150 &&
      Number.isFinite(receivedAt) && now - receivedAt >= 0 && now - receivedAt < 1800;
    const rows = new Map((Array.isArray(s?.joints) ? s.joints : []).filter(row => IDS.includes(row?.id)).map(row => [row.id, row]));
    const valid = row => fresh && row?.fresh === true && row.valid === true && recordValid(row);
    const missing = IDS.filter(id => !valid(rows.get(id)));
    const metaFresh = row => row && Number.isFinite(row.metadata_age_ms) && row.metadata_age_ms >= 0 && row.metadata_age_ms <= 750;
    const offFlag = s?.all_torque_off_verified !== undefined ? s.all_torque_off_verified === true :
      Number.isInteger(s?.mcu_flags) ? Boolean(s.mcu_flags & 16) : true;
    const allOff = fresh && offFlag && IDS.every(id => metaFresh(rows.get(id)) && rows.get(id).torque_enabled === false);
    const stable = fresh && IDS.every(id => valid(rows.get(id)) && rows.get(id).stable === true);
    const policyLocked = fresh && s.policy_locked === true;
    const row = rows.get(selectedId);
    const idle = s?.armed?.active === false && !s.session_active && !s.operation;
    const capture = policyLocked && allOff && stable && idle && s.capture?.ready === true;
    const groupIds = GROUPS[group] || [];
    const groupState = s?.groups?.[group];
    const groupMissing = groupIds.filter(id=>!valid(rows.get(id)));
    const groupStable = fresh && groupIds.length===5 && groupIds.every(id=>valid(rows.get(id))&&rows.get(id).stable===true);
    const groupCapture = policyLocked && allOff && groupStable && idle && groupState?.ready===true &&
      Array.isArray(groupState.ids) && groupState.ids.length===5 && groupIds.every(id=>groupState.ids.includes(id));
    const selected = policyLocked && valid(row) && (row.sign === 1 || row.sign === -1);
    const motionSelected=selected&&motionSafe(row,s);
    return {s, connected, interlocked, supported, fresh, rows, valid, missing, allOff, stable, policyLocked, capture, selected, motionSelected, row, idle,
      group,groupIds,groupState,groupMissing,groupStable,groupCapture};
  }

  function createController(options) {
    const now = options.now || Date.now;
    const change = options.change || (() => {});
    let envelope = null, receivedAt = null, id = 10, group = 'left', page = 'imu', hidden = false;
    let runtime = null, runtimeReceivedAt = null, allAcknowledged = false;
    let statusRuntimeInstance = null, statusOffVerified = false;
    let acknowledged = false, clear = false, phase = 'support', armed = null, busy = false;
    let generation = 0, operation = null, lastMessage = '', records = {}, pollGeneration = 0;
    const pendingHeartbeats = new Set();
    let heartbeatAckOrder = 0, lastHeartbeatAt = -Infinity, lastHeartbeatAck = null, pollBusy = false;
    let lastSafePollAt = -Infinity;
    let requestOrder = 0, acceptedOrder = 0;
    let homeAcknowledged=false,homeSession=null,homePending=false,homeStartedAt=0,homeStopRequested=false;
    let homeContinuationAt=null;
    const homeDeadlineAlive=()=>!!homeSession&&(homeSession.holding===true||now()<homeSession.deadline);
    // This measures this page's actual request submissions, not device receipt.
    // Host keeps its independent 400 ms received-heartbeat lease; MCU keeps 250 ms.
    const homeContinuationAlive=()=>Number.isFinite(homeContinuationAt)&&now()-homeContinuationAt>=0&&now()-homeContinuationAt<400;
    const view = () => {
      const check = assess(envelope, now(), receivedAt, id, group);
      const active = page === 'servo' && !hidden;
      const r=runtime?.status;
      const runtimeOnline = !hidden && !runtime?.error && !runtime?.paused && r?.mode==='read_only' &&
        r.host_read_only===true && r.no_rpc_writes===true && Number.isFinite(r.feedback_age_ms) &&
        r.feedback_age_ms>=0 && r.feedback_age_ms<150 && uint32(r.mcu_seq) && Number.isFinite(r.received) && r.received>0 &&
        Number.isFinite(runtimeReceivedAt) && now()-runtimeReceivedAt>=0 && now()-runtimeReceivedAt<1800 &&
        (!Number.isFinite(runtime.checkedAt) || now()-runtime.checkedAt>=0&&now()-runtime.checkedAt<1800);
      const communicationIds = runtimeOnline && Array.isArray(r.servo_ids)?r.servo_ids.filter(id=>IDS.includes(id)):[];
      const homeOwned=!!homeSession||homePending,homeActive=homeOwned||homeStopRequested||homeUnreleased(check.s?.home);
      const liveArm = active && !check.interlocked && check.motionSelected && acknowledged && clear && armed?.id === id &&
        armed.boot === check.s.boot_id && now() < armed.deadline && Number.isFinite(lastHeartbeatAck) &&
        now()-lastHeartbeatAck < 400 && check.s.armed?.active === true && check.s.armed.id === id;
      const clickReady=active&&!check.interlocked&&!busy&&!homeActive;
      const savedGroups=Object.fromEntries(Object.keys(GROUPS).map(name=>[name,groupRecordsSaved(records,name)]));
      return {...check, id, page, hidden, active, busy, phase, acknowledged, allAcknowledged, clear, records,savedGroups,groupSaved:savedGroups[group],runtimeOnline,communicationIds,
        homeAcknowledged,homeOwned,homeActive,homePhase:check.s?.home?.phase||'off',
        homeReady:homePlanReady(check),canHome:active&&!check.interlocked&&!busy&&!armed&&!homeActive&&homeAcknowledged&&homePlanReady(check),
        canClickHome:clickReady&&!armed&&check.connected&&check.supported&&check.s?.version===HOME_VERSION&&check.idle,
        canClickOff:clickReady&&check.policyLocked,
        canClickCapture:clickReady&&phase==='align'&&!armed&&check.groupCapture,
        canClickArm:clickReady&&check.motionSelected&&check.row.torque_enabled===false&&check.idle&&!armed,
        homeExpires:homeSession?(homeSession.holding?null:Math.max(0,Math.ceil((homeSession.deadline-now())/1000))):0,
        homeLinkPending:!!homeSession&&now()-lastHeartbeatAck>=400,
        controllerConnected:check.connected||runtimeOnline,
        canOff: active && !check.interlocked && !busy && !homeActive && check.policyLocked && acknowledged,
        canCapture: active && !check.interlocked && !busy && !homeActive && phase === 'align' && acknowledged && !armed && check.groupCapture,
        canCaptureAll: active && !check.interlocked && !busy && !homeActive && phase === 'align' && allAcknowledged && !armed && check.capture &&
          (!nativeContract(check.s)||IDS.every(value=>!recordValid(records[value]))),
        canArm: active && !check.interlocked && !busy && !homeActive && acknowledged && clear && check.motionSelected && check.row.torque_enabled === false && check.idle && !armed,
        canJog: active && !busy && Boolean(liveArm),
        canHeartbeat: Boolean(liveArm)||Boolean(active&&!check.interlocked&&homeSession&&homeAcknowledged&&check.fresh&&check.s.boot_id===homeSession.boot&&check.s.home?.supported===true&&check.s.home.active===true&&homeDeadlineAlive()&&homeContinuationAlive()),
        canRead: active && !busy && !homeActive && check.fresh,
        expires: liveArm ? Math.max(0, Math.ceil((armed.deadline - now()) / 1000)) : 0,
        message: lastMessage,
      };
    };
    const notify = () => change(view());
    const fail = message => { lastMessage = message; notify(); };

    async function sendCancel(session) {
      try {
        const result = await options.request('cancel', session ? {session} : {}, {keepalive:true});
        return result?.ok === true;
      } catch { return false; }
    }
    async function sendHomeStop(session) {
      try{return await options.request('home_stop',session?{session}:{},{keepalive:true});}catch{return null;}
    }
    function invalidate(reason, force = false) {
      const previous = armed;
      const pendingArm = operation?.op === 'arm';
      const previousHome=homeSession,pendingHome=homePending;
      generation += 1;
      armed = null; lastHeartbeatAck = null;
      homeSession=null;homePending=false;homeAcknowledged=false;homeContinuationAt=null;
      if(previousHome||pendingHome){homeStopRequested=true;void sendHomeStop(previousHome?.session);}
      if (previous || pendingArm || force) {
        acknowledged = false; allAcknowledged=false; clear = false;
        void sendCancel(previous?.session);
      }
      if (reason) lastMessage = reason;
      notify();
    }
    function accept(value, order = ++requestOrder) {
      if(!homeSession&&order < acceptedOrder)return;
      const previous = envelope?.status;
      // Request completion order is not source order. A delayed successful ACK
      // cannot replace newer HOME evidence or extend its freshness/continuation.
      if(homeSession&&previous?.boot_id===value?.status?.boot_id&&uint32(previous?.sequence)&&uint32(value?.status?.sequence)){
        const step=(value.status.sequence-previous.sequence)>>>0;
        if(step===0||step>=0x80000000)return;
      }else if(order < acceptedOrder)return;
      acceptedOrder = Math.max(acceptedOrder,order);
      envelope = value;
      receivedAt = now();
      statusRuntimeInstance = runtime?.status?.app_instance_id || null;
      const check = assess(envelope, now(), receivedAt, id, group);
      statusOffVerified = check.allOff && check.policyLocked && !check.interlocked;
      const restarted = previous && check.fresh && (previous.boot_id !== check.s.boot_id ||
        (uint32(previous.sequence) && check.s.sequence < previous.sequence));
      if (check.s?.records && typeof check.s.records === 'object') records = check.s.records;
      if(homeStopRequested&&homeReleased(check)){homeStopRequested=false;lastMessage='已停止、已卸力。';}
      if(homeSession){
        const h=check.s?.home;
        if(check.interlocked||restarted||!check.fresh||!check.policyLocked||h?.supported!==true||h.contract_revision!==HOME_CONTRACT_REVISION||check.s.boot_id!==homeSession.boot||h.native_sha256!==homeSession.nativeSha){
          invalidate('归位反馈中断；已请求停止。');
        }else if(homeReleased(check)){
          homeSession=null;homePending=false;homeAcknowledged=false;lastHeartbeatAck=null;homeContinuationAt=null;
          lastMessage=h.arrival_verified===true?'已归位、已卸力。':'已停止、已卸力。';
        }else if(h.phase==='holding'&&!completeHomeHold(h)){
          invalidate('到位保持证据不完整；已请求停止。');
        }else if(homeSession.holding&&!['holding','releasing'].includes(h.phase)){
          invalidate('保持状态异常改变；已请求停止。');
        }else if(h?.active!==true||!['prepared','enabling','moving','holding','releasing'].includes(h.phase)){
          invalidate('归位未确认；已请求停止。');
        }else if(completeHomeHold(h)){
          homeSession.holding=true;
        }
      }
      if (check.interlocked || !check.policyLocked || restarted || (armed && (!check.motionSelected || check.s.armed?.active !== true || check.s.armed.id !== id))) {
        if (armed || operation?.op === 'arm') invalidate(check.interlocked?'无线更新尚未确认；小步已取消。':restarted ? '主控已重启；小步已取消。' : '反馈中断；小步已取消。');
        acknowledged = false;allAcknowledged=false;
        clear = false;
      }
      if (restarted) { phase = 'support'; acknowledged = false; allAcknowledged=false; clear = false; }
      if(check.allOff && !armed && !operation)phase='align';
      notify();
    }
    function safeIdleObserved() {
      // Limit only redundant full-status reads. Recheck the current stream on
      // every timer tick; action gates and the board retain their own checks.
      if(armed||busy||operation||homeSession||homePending||homeStopRequested)return false;
      const check=assess(envelope,now(),receivedAt,id,group);
      const s=envelope?.status,r=runtime?.status,elapsed=now()-runtimeReceivedAt;
      const home=s?.home;
      const homeSettled=home?.active===false&&(
        home.phase==='off'&&home.error===0||
        home.phase==='fault'&&home.error===8&&home.release_verified===true&&
        home.progress?.off_mask===(1<<IDS.length)-1&&home.progress?.enabled_mask===0);
      return statusOffVerified&&check.fresh&&check.allOff&&check.policyLocked&&check.idle&&
        envelope?.connection?.online===true&&envelope.connection.update_interlocked!==true&&
        s?.all_torque_off_verified===true&&s.torque_may_be_on===false&&s.mcu_state===1&&
        s.session_active===false&&s.armed?.active===false&&!s.operation&&
        homeSettled&&
        s.sequence_calibration?.active===false&&s.sequence_calibration.phase==='off'&&s.sequence_calibration.error===0&&
        !runtime?.error&&!runtime?.paused&&r?.mode==='wireless_calibration'&&
        typeof statusRuntimeInstance==='string'&&!!statusRuntimeInstance&&r.app_instance_id===statusRuntimeInstance&&
        uint32(r.mcu_seq)&&Number.isFinite(r.received)&&r.received>0&&
        Number.isFinite(runtimeReceivedAt)&&Number.isFinite(elapsed)&&elapsed>=0&&
        Number.isFinite(r.feedback_age_ms)&&r.feedback_age_ms>=0&&r.feedback_age_ms+elapsed<150&&
        r.calibration_supported===true&&r.policy_locked===true&&r.all_torque_off_verified===true&&
        r.torque_may_be_on===false&&r.session_active===false&&r.enabled===false&&r.mcu_state===1&&
        Array.isArray(r.servo_ids)&&new Set(r.servo_ids).size===IDS.length&&IDS.every(value=>r.servo_ids.includes(value));
    }
    async function refresh() {
      if(pollBusy || hidden || safeIdleObserved()&&now()-lastSafePollAt<1000)return;
      pollBusy = true;
      const mark = ++pollGeneration;
      const order = ++requestOrder;
      try {
        const value = await options.readStatus();
        if (mark !== pollGeneration || hidden) return;
        accept(value,order);
      } catch {
        if (mark !== pollGeneration || hidden) return;
        accept({connection:{online:false,error:'offline'},status:null},order);
      } finally {
        pollBusy = false;
        lastSafePollAt = safeIdleObserved()?now():-Infinity;
      }
    }
    function acceptRuntime(value) { runtime=value;runtimeReceivedAt=now();notify(); }
    function selectGroup(nextGroup) {
      if(!Object.hasOwn(GROUPS,nextGroup)||nextGroup===group)return;
      invalidate(armed||operation?.op==='arm'?'已换组；小步已取消。':'');
      group=nextGroup;if(!GROUPS[group].includes(id))id=GROUPS[group][0];acknowledged=false;allAcknowledged=false;clear=false;lastMessage='';notify();
    }
    function select(nextId) {
      if (!IDS.includes(nextId) || nextId === id) return;
      invalidate(armed || operation?.op === 'arm' ? '已换关节；小步已取消。' : '');
      id = nextId; group=groupForId(id);
      acknowledged = false;allAcknowledged=false;
      clear = false;
      notify();
    }
    function setPage(value) {
      page = value;
      if (page !== 'servo') {
        invalidate(armed || operation?.op === 'arm' ? '小步已取消。' : '');
        acknowledged = false; allAcknowledged=false; clear = false;
      }
      notify();
    }
    function visibility(value) {
      hidden = Boolean(value);
      pollGeneration += 1;
      if (hidden) {
        invalidate(armed || operation?.op === 'arm' ? '页面已隐藏；小步已取消。' : '');
        acknowledged = false; allAcknowledged=false; clear = false; envelope = null; receivedAt = null;runtime=null;runtimeReceivedAt=null;
      }
      notify();
    }
    function acknowledge(value) {
      acknowledged = Boolean(value);
      if (!acknowledged) {allAcknowledged=false;invalidate(armed || operation?.op === 'arm' ? '确认已撤销；小步已取消。' : '');}
      notify();
    }
    function acknowledgeAll(value) { allAcknowledged=Boolean(value);notify(); }
    function acknowledgeHome(value){
      homeAcknowledged=value===true;
      if(!homeAcknowledged&&(homeSession||homePending))invalidate('归位确认已撤销；已请求停止。');
      notify();
    }
    function clearance(value) {
      clear = Boolean(value);
      if (!clear) invalidate(armed || operation?.op === 'arm' ? '小步已取消。' : '');
      notify();
    }
    function tick() {
      const check = view();
      if(homeSession&&(!check.canHeartbeat||!homeDeadlineAlive()||!homeContinuationAlive()))invalidate('归位授权已结束；已请求停止。');
      if(homePending&&now()-homeStartedAt>=30000)invalidate('归位准备回执未知；已请求停止。');
      if(!check.fresh&&!homeSession)homeAcknowledged=false;
      if (armed && (!check.canJog && !busy || now() >= armed.deadline || !check.fresh || now()-lastHeartbeatAck >= 400)) {
        invalidate('授权已结束；小步已取消。');
        acknowledged = false; allAcknowledged=false; clear = false;
      }
      if (!check.fresh && (acknowledged||allAcknowledged)) { acknowledged = false; allAcknowledged=false; clear = false; }
      notify();
    }
    async function heartbeat() {
      const check = view();
      if (!check.canHeartbeat || now()-lastHeartbeatAt < 90) return;
      const isHome=!!homeSession,mark = generation, session = isHome?homeSession.session:armed.session, selected = id;
      // Keep the existing 400 ms client watchdog while allowing local HTTP
      // replies to overlap. Motion requests remain single, never retried.
      const inFlight=[...pendingHeartbeats].filter(item=>item.mark===mark&&item.session===session).length;
      if(inFlight >= (isHome?4:1))return;
      const order = ++requestOrder;
      const pending={mark,session,sentAt:now()};pendingHeartbeats.add(pending);lastHeartbeatAt=pending.sentAt;
      if(isHome)homeContinuationAt=pending.sentAt;
      try {
        const result = await options.request(isHome?'home_heartbeat':'heartbeat',isHome?{session,home_contract_revision:HOME_CONTRACT_REVISION}:{id:selected,session});
        if (mark !== generation || (isHome?homeSession?.session:armed?.session) !== session) return;
        if(!isHome&&order < heartbeatAckOrder)return;
        if (result?.ok !== true) throw Error('heartbeat_failed');
        if(isHome&&!view().canHeartbeat){
          invalidate('归位心跳超时；已请求停止。');return;
        }
        heartbeatAckOrder=Math.max(heartbeatAckOrder,order);
        lastHeartbeatAck = now();
        if (result.status) accept({connection:{...envelope?.connection,online:true},status:result.status},order);
      } catch {
        if (order>=heartbeatAckOrder&&mark === generation && (isHome?homeSession?.session:armed?.session) === session) invalidate(isHome?'归位心跳中断；已请求停止。':'心跳中断；小步已取消。');
      } finally { pendingHeartbeats.delete(pending); }
    }
    async function action(op, modelDelta) {
      const check = view();
      if (op === 'cancel') {
        if(check.homeActive||check.s?.home?.phase==='fault'&&check.s.home.release_verified!==true){
          const previous=homeSession;generation++;homeSession=null;homePending=false;homeStopRequested=true;homeAcknowledged=false;armed=null;lastHeartbeatAck=null;acknowledged=false;allAcknowledged=false;clear=false;notify();
          const result=await sendHomeStop(previous?.session);
          if(result?.status)accept({connection:{...envelope?.connection,online:true},status:result.status});
          const release=result?.ok===true&&homeReleased(assess({connection:{online:true},status:result.status},now(),now(),id,group));
          fail(release?'已停止、已卸力。':'停止待确认；请保持托稳。');await refresh();return release;
        }
        const previous = armed;
        generation += 1; armed = null; lastHeartbeatAck = null; notify();
        acknowledged = false; allAcknowledged=false; homeAcknowledged=false; clear = false;
        const ok = await sendCancel(previous?.session);
        fail(ok ? '停止已确认。' : '未收到停止回执；请保持托稳。');
        await refresh();
        return ok;
      }
      if(op==='home')return homeAction(check);
      if (busy || !check.active) return false;
      const allowed = {off_all:check.canOff,capture_group:check.canCapture,capture_all:check.canCaptureAll,arm:check.canArm,jog:check.canJog,read:check.canRead}[op];
      if (!allowed || op === 'jog' && ![-0.966796875,-0.263671875,0.263671875,0.966796875].includes(modelDelta)) return false;
      const mark = generation, selected = id, selectedGroup=group, boot = check.s.boot_id;
      const order = ++requestOrder;
      const payload = op === 'off_all' ? {supported:true} : op === 'capture_group' ? {group,supported:true,pose_confirmed:true} : op === 'capture_all' ? {supported:true,all_aligned:true} :
        op === 'arm' ? {id,supported:true,clear:true} : op === 'jog' ? {id,session:armed.session,model_delta_deg:modelDelta} : {id};
      operation = {op,mark}; busy = true; notify();
      if(op==='off_all'){armed=null;lastHeartbeatAck=null;}
      try {
        const result = await options.request(op,payload,{guard:()=>mark===generation&&selected===id&&selectedGroup===group&&!hidden&&page==='servo'});
        if (result?.ok !== true) throw Object.assign(new Error(result?.error || result?.detail || '操作未完成'),{result});
        if (mark !== generation || selected !== id || selectedGroup!==group || hidden || page !== 'servo') {
          if (op === 'arm' && typeof result.session === 'string') await sendCancel(result.session);
          return false;
        }
        if (op === 'arm') {
          const state = result.status;
          const ms = Math.min(30000, Number(result.expires_ms ?? state?.armed?.expires_ms));
          if (typeof result.session !== 'string' || !result.session || !Number.isFinite(ms) || ms <= 0 ||
              state?.boot_id !== boot || state?.armed?.active !== true || state?.armed?.id !== selected) {
            await sendCancel(typeof result.session === 'string' ? result.session : undefined);
            throw new Error('授权回执不完整；已取消。');
          }
          armed = {id:selected,session:result.session,boot,deadline:now()+ms};
          lastHeartbeatAck = now(); lastHeartbeatAt = -Infinity;
          if (result.status) accept({connection:{...envelope?.connection,online:true},status:result.status},order);
          lastMessage = '仅这颗允许小步。';
        } else if (op === 'off_all') {
          armed = null; lastHeartbeatAck = null; clear = false; acknowledged = false;allAcknowledged=false;
          if (result.status) accept({connection:{...envelope?.connection,online:true},status:result.status},order);
          if (!view().allOff) throw new Error('卸力尚未全部确认；保持托稳。');
          phase = 'align'; lastMessage = '已卸力。摆好'+GROUP_LABELS[group]+'后'+(groupRecordsSaved(records,group)?'重新记录':'记录')+'零位。';
        } else if (op === 'capture_group') {
          const state=result.status, next=state?.records, nextGroup=state?.groups?.[selectedGroup];
          const captured=result.captured_ids, required=GROUPS[selectedGroup];
          if(result.op!=='capture_group' || result.group!==selectedGroup || result.saved_count!==5 || !Array.isArray(captured) ||
              required.some(value=>!captured.includes(value)||!state?.captured_ids?.includes(value)) || nextGroup?.recorded!==true ||
              required.some(value=>!recordValid(nextGroup.records?.[value])||!sameRecord(nextGroup.records[value],next?.[value])) ||
              !next || required.some(value=>!recordValid(next[value])) || state?.group_file?.verified!==true || !receiptContext(state) ||
              result.complete!==state.complete || result.complete===true && (result.published!==true || state.published!==true || receiptFile(state)?.verified!==true || typeof state.group_file.sha256!=='string' ||
                receiptFile(state).group_records_sha256!==state.group_file.sha256 || !nativeFileVerified(state) || nativeContract(state)&&state.native_published!==true ||
                IDS.some(value=>!captured.includes(value)||!recordValid(next[value])))) {
            throw new Error('这组记录回执未齐全；保留原记录。');
          }
          records=next;accept({connection:{...envelope?.connection,online:true},status:state},order);
          acknowledged=false;allAcknowledged=false;
          lastMessage=result.complete===true?'15 / 15 零位已保存；可重新记录任一组。显示方向待核对。':GROUP_LABELS[selectedGroup]+' 5 / 5 已记录；可重新记录这一组。';
        } else if (op === 'capture_all') {
          const ids = result.status?.captured_ids;
          const next = result.status?.records;
          if (result.status?.complete !== true || !Array.isArray(ids) || IDS.some(value=>!ids.includes(value)) ||
              !next || IDS.some(value=>!recordValid(next[value])) || receiptFile(result.status)?.verified !== true || !receiptContext(result.status) ||
              nativeContract(result.status)&&(result.status.native_published!==true||result.status.published!==true||!nativeFileVerified(result.status)||receiptFile(result.status).group_records_sha256!==result.status.group_file.sha256)) {
            throw new Error('记录回执未齐全；保留原记录。');
          }
          records = next;
          accept({connection:{...envelope?.connection,online:true},status:result.status},order);
          acknowledged = false;allAcknowledged=false;
          lastMessage = '15 / 15 原生编码已记录。掉电/方向/模型应用待核验。';
        } else {
          if (result.status) accept({connection:{...envelope?.connection,online:true},status:result.status},order);
          lastMessage = op === 'jog' ? '小步已完成。' : '位置已读取。';
        }
        return true;
      } catch (error) {
        if (op === 'jog' || op === 'arm' && !localUnsent(error.result)) invalidate('', true);
        if (['capture_all','capture_group','off_all'].includes(op)) {acknowledged=false;allAcknowledged=false;}
        lastMessage = formatError(error.result || error.message);
        return false;
      } finally {
        operation = null; busy = false; notify();
      }
    }
    async function userAction(op,modelDelta){
      // Intent comes from the action button itself, never from page load/polling.
      const check=view();
      const allowed={home:check.canClickHome,off_all:check.canClickOff,capture_group:check.canClickCapture,arm:check.canClickArm,
        jog:check.canJog,read:check.canRead,cancel:true}[op];
      if(!allowed)return false;
      if(op==='home')return requestHome();
      if(['off_all','capture_group','arm'].includes(op))acknowledged=true;
      if(op==='arm')clear=true;
      return action(op,modelDelta);
    }
    async function requestHome(){
      if(!view().canClickHome)return false;
      const mark=generation,signal=AbortSignal.timeout(1500);
      const timeout=new Promise((_,reject)=>signal.addEventListener('abort',()=>reject(Error('status_timeout')),{once:true}));
      busy=true;lastMessage='正在读取归位状态…';notify();
      try{
        // Only read-only refreshes may repeat. A fresh, fully checked snapshot
        // authorizes exactly one prepare; this click never retries a motion.
        for(let attempt=0;attempt<3;attempt++){
          const order=++requestOrder;
          const value=await Promise.race([options.readStatus({signal}),timeout]);
          if(mark!==generation||hidden||page!=='servo')return false;
          accept(value,order);
          const check=view();
          if(!check.connected){lastMessage='无线未连接';return false;}
          if(check.interlocked){lastMessage='更新尚未确认';return false;}
          if(check.homeReady&&!check.homeActive&&!armed){
            busy=false;homeAcknowledged=true;return await homeAction(view());
          }
          if(check.fresh){lastMessage=check.homeActive?'已有归位操作':homeBlockedNote(check);return false;}
        }
        lastMessage='暂未取得新鲜反馈';return false;
      }catch{
        if(mark===generation&&!hidden&&page==='servo')lastMessage='暂未取得新鲜反馈';
        return false;
      }finally{busy=false;if(mark!==generation&&lastMessage==='正在读取归位状态…')lastMessage='';notify();}
    }
    async function homeAction(check){
      if(!check.canHome)return false;
      const mark=generation,boot=check.s.boot_id,nativeSha=check.s.home.plan.native_sha256;
      homePending=true;homeStopRequested=false;homeStartedAt=now();busy=true;operation={op:'home_prepare',mark};lastMessage='正在核对归位起点…';notify();
      let token=null;
      try{
        const prepareOrder=++requestOrder;
        const prepared=await options.request('home_prepare',{native_sha256:nativeSha,supported:true,clear:true,home_contract_revision:HOME_CONTRACT_REVISION},
          {guard:()=>mark===generation&&!hidden&&page==='servo'&&homeAcknowledged});
        token=typeof prepared?.session==='string'?prepared.session:null;
        if(mark!==generation||hidden||page!=='servo'||!homeAcknowledged){if(token)await sendHomeStop(token);return false;}
        const s=prepared?.status,h=s?.home,ms=Math.min(30000,Number(prepared?.expires_ms??h?.expires_ms));
        const evidence=assess({connection:{online:true},status:s},now(),now(),id,group);
        if(prepared?.ok!==true||prepared.op!=='home_prepare'||!token||!Number.isFinite(ms)||ms<=0||now()-homeStartedAt>=30000||
          !evidence.fresh||!evidence.policyLocked||!evidence.allOff||s.boot_id!==boot||h?.contract_revision!==HOME_CONTRACT_REVISION||h?.active!==true||h.phase!=='prepared'||h.native_sha256!==nativeSha)
          throw Object.assign(new Error('归位准备回执未齐全'),{result:prepared});
        homeSession={session:token,boot,nativeSha,deadline:Math.min(homeStartedAt+30000,now()+ms),holding:false};homePending=false;lastHeartbeatAck=now();lastHeartbeatAt=-Infinity;homeContinuationAt=now();
        accept({connection:{...envelope?.connection,online:true},status:s},prepareOrder);
        if(!homeSession||mark!==generation)return false;
        void heartbeat();
        if(!homeSession||mark!==generation||!view().canHeartbeat)return false;
        homeSession.deadline=now()+HOME_MOTION_LIMIT_MS;
        operation={op:'home_run',mark};lastMessage='正在逐颗缓慢回中；随时可停止。';notify();const runOrder=++requestOrder;
        const result=await options.request('home_run',{session:token,native_sha256:nativeSha,home_contract_revision:HOME_CONTRACT_REVISION});
        if(mark!==generation||hidden||page!=='servo'){await sendHomeStop(token);return false;}
        const runEvidence=assess({connection:{online:true},status:result?.status},now(),now(),id,group),rh=result?.status?.home;
        if(result?.ok!==true||result.op!=='home_run'||!runEvidence.fresh||!runEvidence.policyLocked||result.status.boot_id!==boot||rh?.native_sha256!==nativeSha||
          !(rh.active===true&&['enabling','moving','holding','releasing'].includes(rh.phase)||homeReleased(runEvidence)))throw Object.assign(new Error('归位未确认'),{result});
        if(result.status)accept({connection:{...envelope?.connection,online:true},status:result.status},runOrder);
        return true;
      }catch(error){
        if(mark===generation){
          const unsent=operation?.op==='home_prepare'&&!token&&(errorReason(error.result)==='command_route_unavailable'||localUnsent(error.result));
          homeSession=null;homePending=false;homeStopRequested=!unsent;homeAcknowledged=false;lastHeartbeatAck=null;generation++;
          if(unsent){lastMessage=formatError(error.result);await refresh();return false;}
          const stopped=await sendHomeStop(token);if(stopped?.status)accept({connection:{...envelope?.connection,online:true},status:stopped.status});
          const released=stopped?.ok===true&&homeReleased(assess({connection:{online:true},status:stopped.status},now(),now(),id,group));
          lastMessage=(released?'归位未完成，已卸力。':'归位未确认；保持托稳。')+(error.result?formatError(error.result):'');
        }
        return false;
      }finally{operation=null;busy=false;notify();}
    }
    return {view,accept,acceptRuntime,refresh,select,selectGroup,setPage,visibility,acknowledge,acknowledgeAll,acknowledgeHome,clearance,tick,heartbeat,action,userAction};
  }

  function errorReason(value){
    const detail=value?.detail&&typeof value.detail==='object'?value.detail:value;
    return detail?.reason||detail?.code||(typeof detail?.error==='string'?detail.error:typeof value==='string'?value:null);
  }
  function formatError(value) {
    const raw = typeof value === 'string' ? value : value?.detail ?? value?.error ?? value?.message;
    const detail = typeof raw === 'object' && raw ? raw : value;
    const fault=homeFaultNote(detail?.status||value?.status);
    if(fault)return fault;
    const ids = [...new Set([...(Array.isArray(detail?.blocked_ids)?detail.blocked_ids:detail?.status?.capture?.blocked_ids||[]),
      ...(Array.isArray(detail?.global_torque_blocked_ids)?detail.global_torque_blocked_ids:[])])];
    const reason = typeof raw === 'string' ? raw : detail?.reason || detail?.message || '未完成，请检查反馈。';
    const labels = {home_contract_mismatch:'归位页面已更新，请刷新后重新操作。',offline:'无线未连接。',feedback_stale:'反馈过期，请保持托稳。',unsupported:'主控校准功能尚未就绪。',
      unstable:'仍有晃动，放稳后重试。',torque_enabled:'尚未全部卸力。',invalid_position:'位置编码待核对。',
      pose_required:'先按图摆好这组关节。',all_torque_off_required:'先确认全机卸力。',global_torque_unverified:'全机卸力尚未确认。',session_expired:'授权已结束。',session_invalid:'授权已取消。',session_mismatch:'归位会话已结束；查看故障反馈。',
      command_route_unavailable:'归位未发送：命令通道正在恢复，请稍后重新点归位。',command_delivery_unknown:'命令回执中断；正在停止本次操作。'};
    labels.local_session_expired='本地服务已重启；本次操作未发送，请重新点击。';
    labels.local_session_unavailable='本地服务正在恢复；本次操作未发送，请稍后重新点击。';
    labels.local_request_cancelled='页面状态已改变；本次操作未发送。';
    return (Array.isArray(ids) && ids.length ? 'ID '+ids.join('、')+'：' : '') + (labels[errorReason(value)] || labels[reason] || reason);
  }

  const exported = {IDS,GROUPS,GROUP_LABELS,SCHEMA,STATUS_URL,ACTION_URL,LOCAL_SESSION_URL,NATIVE_VERSION,PREVIOUS_HOME_VERSION,HOME_VERSION,NATIVE_DOMAIN,recordValid,groupRecordsSaved,motionSafe,observedPosition,jointReadout,homePositionAllowed,homeTargetMatches,homeBlockedNote,homePlanReady,homeReleased,homeIdleNote,homeFaultNote,completeHomeHold,assess,createController,createRequest,formatError};
  if (typeof module === 'object' && module.exports) { module.exports = exported; return; }
  scope.DuckWirelessCalibration = exported;
  scope.DUCK_WIRELESS_CALIBRATION = true;
  const get = id => document.getElementById(id);
  if (!get('wireless-calibration')) return;
  const request = createRequest({fetch:(...args)=>fetch(...args),getToken:()=>scope.COMMISSION_TOKEN,setToken:value=>{scope.COMMISSION_TOKEN=value;}});
  const readStatus = async (extra={}) => {
    const response = await fetch(STATUS_URL,{cache:'no-store',signal:extra.signal});
    if (!response.ok) throw Error('offline');
    return response.json();
  };
  const servoDots=[];
  let paintedAt=-Infinity,paintedKey='',lastBroadcastStatus,lastBroadcastRecords,lastBroadcastKey='',motionKey='';
  const setText=(node,value)=>{if(node&&node.textContent!==value)node.textContent=value;};
  function paintPanel(v) {
    get('wl-connection').textContent = v.interlocked?'无线更新尚未确认':v.runtimeOnline&&!v.fresh?'主控已连接·校准程序待更新':!v.connected ? '无线离线' : !v.supported ? '主控已连接·校准程序待更新' : !v.fresh ? '等待新反馈' : '无线已连接';
    get('wl-connection').dataset.state = v.fresh ? 'ok' : 'unknown';
    if(get('wl-home')){get('wl-home').disabled=!v.canClickHome||scope.duckSequenceCalibration?.needsStop()===true;get('wl-home').textContent='回正';}
    get('wl-stop').textContent='停止/卸力';
    if(get('wl-home-state')){
      const h=v.s?.home;
      get('wl-home-state').title=homeFaultNote(v.s)||(h?.phase==='fault'?'上次归位错误 '+(h.error??'未知')+(h.error===13?'：卸力校验未通过，不能仅据此确定最初失败原因。':'。'):'');
      get('wl-home-state').textContent=v.interlocked?'程序更新中':!v.connected?'等待无线连接':v.s?.version!==HOME_VERSION||!h?'归位程序待更新':
        v.homeActive?(!v.fresh||h.supported!==true?'停止待确认；保持托稳，可停止':{preparing:'正在核对起点…',prepared:'起点已核对，准备归位…',enabling:homeProgressNote(h),moving:homeProgressNote(h),
          holding:completeHomeHold(h)?'已到位 · 保持中（5°）':'到位待确认；可停止',releasing:'正在卸力；可停止',off:v.allOff?'已卸力；上次归位未完成':'归位未完成；可停止'}[h.phase]||'归位状态待确认；可停止'):
        h.supported!==true?'等待新鲜归位反馈':
        homeIdleNote(v);
    }
    get('wl-confirm-label').textContent = v.groupSaved?'可重新校准'+GROUP_LABELS[v.group]+'；摆好后重新记录，只替换这一组':v.phase === 'align' ? '托稳机身，摆好'+GROUP_LABELS[v.group]+'后点记录' : '托稳机身后操作';
    get('wl-capture-all').textContent=(v.groupSaved?'重新记录':'记录')+GROUP_LABELS[v.group]+'零位';
    if(get('wl-group-label'))get('wl-group-label').textContent=GROUP_LABELS[v.group];
    if(get('wl-group-status'))get('wl-group-status').textContent=v.groupSaved?'已记录':v.s?.records?'待记录':'待读取';
    for(const group of Object.keys(GROUPS)){const node=get('wl-group-'+group);if(node){node.setAttribute('aria-pressed',String(group===v.group));node.dataset.recorded=String(v.savedGroups[group]);node.disabled=v.busy;}}
    get('wl-off-all').disabled = !v.canClickOff;
    get('wl-capture-all').disabled = !v.canClickCapture;
    get('wl-arm').disabled = !v.canClickArm;
    get('wl-read').disabled = !v.canRead;
    get('wl-minus').disabled = get('wl-plus').disabled = !v.canJog;
    get('wl-arm-state').textContent = v.expires ? 'ID '+v.id+' · '+v.expires+' 秒' : '未授权';
    get('wl-direction').textContent = v.row?.sign === 1 ? '模型＋ = 编码＋' : v.row?.sign === -1 ? '模型＋ = 编码−' : '方向映射待确认';
    get('wl-result').textContent = homeFaultNote(v.s) || v.message || (v.interlocked?'无线更新尚未确认。':'');
    for (const node of servoDots) {
      const id = Number(node.dataset.id), row = v.rows.get(id), valid = v.valid(row);
      node.hidden=false;
      const communicating=v.communicationIds.includes(id);
      node.dataset.state = !v.fresh ? communicating?'online':'unknown' : !valid ? 'error' : row.stable !== true ? 'pending' : 'ok';
      node.setAttribute('aria-pressed',String(id===v.id));
      node.title = v.fresh?jointReadout(row,true):'ID '+id+(communicating?' · 通信在线，校准待更新':' · 等待新鲜位置');
    }
    const keys = IDS.filter(id=>recordValid(v.records?.[id]));
    get('wl-saved-count').textContent = keys.length||v.s?.records?keys.length+' / 15':'待读取';
    get('wl-records').textContent = keys.map(id=>id+': '+v.records[id].position).join('　')+(keys.length?'　原生编码仅记录；掉电/方向/模型应用待核验。':'');
    get('wl-records-details').hidden = keys.length===0;
  }
  function render(v) {
    // State ingestion, click gates, watchdog and heartbeat remain synchronous.
    // Only the hidden/unchanged presentation work is suppressed.
    const sharedKey=[v.interlocked,v.supported,v.fresh,v.homeActive,v.homePhase,
      v.s?.home?.supported,v.policyLocked,v.s?.armed?.active].join('|');
    if(sharedKey!==motionKey&&(v.interlocked || v.supported && v.fresh)) {
      motionKey=sharedKey;
      const motion = get('simple-motion');
      setText(motion.querySelector('strong'),v.interlocked?'更新未确认':v.homeActive?v.s?.home?.supported!==true||v.homePhase==='fault'?'停止待确认':({holding:'保持中',moving:'归位中',enabling:'归位中',preparing:'归位准备',prepared:'归位准备',releasing:'卸力中',off:'归位未完成'}[v.homePhase]||'停止待确认'):!v.policyLocked?'未确认锁定':v.s.armed?.active===true?'单颗授权':'已锁定');
      setText(motion.querySelector('small'),v.homeActive?'可随时停止':'无线校准');
      const state=v.interlocked||v.homeActive?'':!v.policyLocked?'fail':v.s.armed?.active===true?'':'pass';
      if(motion.dataset.state!==state)motion.dataset.state=state;
    }
    motionKey=sharedKey;
    const eventKey=[v.fresh,v.id,v.homeActive,v.homeOwned,v.homePhase].join('|');
    if(v.s!==lastBroadcastStatus||v.records!==lastBroadcastRecords||eventKey!==lastBroadcastKey){
      lastBroadcastStatus=v.s;lastBroadcastRecords=v.records;lastBroadcastKey=eventKey;
      scope.dispatchEvent(new CustomEvent('duck:wireless-calibration',{detail:{status:v.s,records:v.records,fresh:v.fresh,selected_id:v.id,homeActive:v.homeActive,homeOwned:v.homeOwned,homePhase:v.homePhase}}));
    }
    if(document.hidden||get('workflow-imu')?.hidden!==false)return;
    const key=[v.s?.sequence,v.id,v.group,v.fresh,v.interlocked,v.connected,v.supported,v.phase,v.busy,
      v.canClickHome,v.canClickOff,v.canClickCapture,v.canClickArm,v.canRead,v.canJog,
      v.homeActive,v.homePhase,v.message,v.expires,v.groupSaved].join('|');
    const now=performance.now();
    if(key===paintedKey&&now-paintedAt<100)return;
    paintedKey=key;paintedAt=now;paintPanel(v);
  }
  const controller = createController({request,readStatus,change:render});
  scope.duckWirelessCalibration = controller;
  for (const id of IDS) {
    const button = document.createElement('button');button.type='button';button.dataset.id=String(id);button.textContent=String(id);
    button.setAttribute('aria-label','选择 ID '+id);button.addEventListener('click',()=>{
      if(scope.DuckWorkflow?.selectJoint)scope.DuckWorkflow.selectJoint(id);
      else get('servo-picker-groups')?.querySelector('button[data-id="'+id+'"]')?.click();
    });get('wl-servo-dots').append(button);servoDots.push(button);
  }
  get('wl-home')?.addEventListener('click',event=>{if(event?.isTrusted)void controller.userAction('home');});
  for (const [id,op,direction] of [['wl-off-all','off_all'],['wl-capture-all','capture_group'],['wl-arm','arm'],
    ['wl-minus','jog',-1],['wl-plus','jog',1],['wl-read','read'],['wl-stop','cancel']]) {
    get(id).addEventListener('click',event=>{if(!event?.isTrusted)return;if(op==='cancel'&&scope.duckSequenceCalibration?.needsStop()){void scope.duckSequenceCalibration.stop();return;}const delta=op==='jog'?direction*Number(get('wl-step-size').value):undefined;void controller.userAction(op,delta);});
  }
  scope.addEventListener('duck:joint',event=>controller.select(event.detail.id));
  scope.addEventListener('duck:live-status',event=>controller.acceptRuntime(event.detail));
  for(const group of Object.keys(GROUPS)){get('wl-group-'+group)?.addEventListener('click',()=>{
    controller.selectGroup(group);
    scope.DuckWorkflow?.selectJoint(controller.view().id);
    scope.dispatchEvent(new CustomEvent('duck:calibration-group',{detail:{group}}));
  });}
  scope.addEventListener('duck:page',event=>{const page=event.detail.page==='imu'?'servo':event.detail.page;controller.setPage(page);if(page==='servo')void controller.refresh();});
  document.addEventListener('visibilitychange',()=>{controller.visibility(document.hidden);if(!document.hidden)void controller.refresh();});
  scope.addEventListener('pagehide',()=>controller.visibility(true));
  get('wl-fine-tools').addEventListener('toggle',()=>{if(!get('wl-fine-tools').open){controller.clearance(false);}});
  setInterval(()=>{controller.tick();if(!document.hidden)void controller.refresh();},250);
  setInterval(()=>{controller.tick();void controller.heartbeat();},100);
  controller.setPage(get('workflow-imu').hidden?'imu':'servo');
  controller.select(Number(scope.DuckWorkflow?.selectedJoint())||10);
  void controller.refresh();
})(typeof window === 'object' ? window : globalThis);
