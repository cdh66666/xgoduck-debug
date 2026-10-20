(function () {
  'use strict';
  const PROFILE = {position: {mode: 4, min: -100, max: 100, nativeMin: 0, nativeMax: 4095}, speed: {mode: 1, min: -100, max: 100, fullScale: 200}, current: {mode: 2, min: -100, max: 100, fullScale: 500}};
  const API = '/api/controller-stream';
  const AXES = [10,11,12,13,14,20,21,22,23,24,30,31,32,33,34];
  const EXPECTED_BUILD = 0x1D10E105;
  function protocolReady(value) { return value?.expected_build_tag === EXPECTED_BUILD && value?.latest?.build_tag === EXPECTED_BUILD && value?.latest?.guard_contract_observed === true; }
  function axis(value) { return integer(value, 10, 34, '关节ID'); }
  function validAxis(value) { return AXES.includes(value); }
  function jointDegrees(native, zero) { return Number.isFinite(native) && Number.isFinite(zero?.native_zero_ticks)
    ? (native - zero.native_zero_ticks) * (zero.sign === -1 ? -1 : 1) * 360 / 4096 : null; }
  function gainExpected(id) { return {kp: id === 34 ? 10 : 6, kd: 20}; }

  function integer(value, min, max, label = '数值') {
    const number = typeof value === 'string' && !value.trim() ? NaN : Number(value);
    if (!Number.isInteger(number) || number < min || number > max) throw new Error(`${label}须为${min}至${max}的整数`);
    return number;
  }
  function target(profile, value) { const p = PROFILE[profile]; if (!p) throw new Error('未知控制种类');
    const number = typeof value === 'string' && !value.trim() ? NaN : Number(value);
    if (!Number.isFinite(number) || number < -100 || number > 100) throw new Error('目标须为−100%至100%');
    return number;
  }
  function percentToNativeValue(profile, value) { const percent = target(profile, value), config = PROFILE[profile];
    return profile === 'position' ? config.nativeMin + (percent + 100) * (config.nativeMax - config.nativeMin) / 200 : percent * config.fullScale / 100; }
  function percentToNative(profile, value) { const exact = percentToNativeValue(profile, value); const native = Math.round(Math.abs(exact)) * (exact < 0 ? -1 : 1); return native === 0 ? 0 : native; }
  function nativeToPercent(profile, value) { if (!Number.isFinite(value)) return null; const config = PROFILE[profile];
    return profile === 'position' ? (value - config.nativeMin) * 200 / (config.nativeMax - config.nativeMin) - 100 : value * 100 / config.fullScale; }
  function seedPercent(profile, native) { const percent = nativeToPercent(profile, native); return percent == null ? null : Math.max(-100, Math.min(100, percent)); }
  function degrees(value) { return Number.isFinite(value) ? value * 360 / 4096 : null; }
  function afterEqual(value, reference) { return Number.isInteger(value) && Number.isInteger(reference) && ((value - reference) >>> 0) < 0x80000000; }
  function raw(value) { return !Number.isInteger(value) ? null : value === 0x8000 ? 0 : value < 0 ? value : value & 0x8000 ? -(value & 0x7fff) : value; }
  function sensor(latest, field, flag, stamp) { return latest?.[flag] === true && latest[stamp] > 0 ? latest[field] : null; }
  function matches(intent, status) {
    const stream = status?.stream || {}, latest = status?.latest || {};
    return intent?.session != null && latest.selected_id === intent.id && stream.id === intent.id && latest.boot_id === intent.boot && latest.session_id === intent.session
      && stream.session === intent.session && stream.begin_request_seq === intent.beginSeq
      && afterEqual(latest.accepted_request_seq, intent.beginSeq);
  }
  function closedMatches(intent, status) {
    const stream = status?.stream || {};
    return status?.latest?.boot_id === intent?.boot && stream.id === intent?.id && stream.session === intent?.session
      && stream.begin_request_seq === intent?.beginSeq && stream.phase === 'closed' && stream.off_verified === true;
  }
  function guardText(reason) {
    const words = {none: '无停机', user_stop: '手动停止', completed: '任务正常结束', lease_expired: '控制租约到期',
      feedback_stale: 'MCU反馈超时', ack_timeout: '应答超时', checksum: '校验错误', device_error: '设备状态报错',
      mode: '模式不符', voltage: '供电范围保护', temperature: '温度保护', fence: '编码范围保护',
      tracking: '持续跟踪偏差', stall: '持续未跟随', current: '电流保护', deadline: '时间上限',
      invalid_request: '请求无效', acquire_requires_off: '接管需先卸力'};
    return words[reason] ? `${words[reason]} (${reason})` : String(reason ?? '—');
  }
  function guardIsError(reason, fault) { return fault === true && !['none', 'user_stop', 'completed'].includes(reason); }
  function recentReply(latest) {
    return latest.ack_status != null ? `最近应答servo_error ${latest.servo_error ?? '—'} · 当前寄存器${latest.query_address ?? '—'} · 无独立时戳`
      : `最近应答servo_error ${latest.servo_error ?? '—'} · 有效性待确认`;
  }
  const helpers = {AXES, EXPECTED_BUILD, protocolReady, validAxis, jointDegrees, gainExpected, PROFILE, integer, target, percentToNativeValue, percentToNative, nativeToPercent, seedPercent, degrees, afterEqual, raw, sensor, matches, closedMatches, guardText, guardIsError, recentReply};
  if (typeof window === 'undefined') { if (typeof module !== 'undefined') module.exports = helpers; return; }
  const $ = id => document.getElementById(id), text = (value, n = 0) => Number.isFinite(value) ? value.toFixed(n) : '—';
  const state = {token: null, status: null, readAt: 0, online: false, generation: 0, intent: null,
    pointer: null, keyHeld: null, controls: new Set(), hbControllers: new Set(),
    sourceKey: null, previous: null, samples: [], guardKey: null, positionSeeded: false, hidden: false,
    id: null, axisTask: null, parameterTask: null, gains: new Map(), signDraft: new Map(), readFailure: null, cancelledIntentSinceRead: false, lastLinkAckAt: 0, heartbeatFailures: 0, pendingExplicitStop: null};
  function notice(message, error = false) { $('notice').textContent = message; $('notice').classList.toggle('error', error); }
  function log(message, error = false) {
    const row = document.createElement('div'); row.className = `event${error ? ' error' : ''}`;
    const clock = document.createElement('time'); clock.textContent = new Date().toLocaleTimeString('zh-CN', {hour12: false});
    const body = document.createElement('span'); body.textContent = message; row.append(clock, body); $('log').prepend(row);
    while ($('log').children.length > 100) $('log').lastChild.remove();
  }
  async function json(path, options = {}, timeout = 1800, controller = new AbortController()) {
    const timer = setTimeout(() => controller.abort(), timeout);
    try {
      const response = await fetch(path, {cache: 'no-store', credentials: 'same-origin', ...options, signal: controller.signal});
      const source = await response.text(); let value;
      try { value = JSON.parse(source); } catch { throw new Error(`HTTP${response.status}：${source.slice(0, 240)}`); }
      if (!response.ok || value.ok === false || value.sent === false && value.error) { const error = new Error(`HTTP${response.status}：${value.error || value.message || source.slice(0, 400)}`); error.httpStatus = response.status; error.response = value; throw error; }
      return value;
    } finally { clearTimeout(timer); }
  }
  async function token() { if (!state.token) state.token = (await json('/api/local-session')).token; return state.token; }
  async function post(body, intent = null, timeout = 1800) {
    if (!['stop','stream_end'].includes(body.op) && !protocolReady(state.status)) throw new Error('主控E105待安装，当前只读；本次未发送');
    const generation = state.generation; const key = await token();
    if (intent && (state.intent !== intent || generation !== state.generation)) throw new Error('旧手势已取消');
    const controller = new AbortController(); if (intent) state.controls.add(controller);
    try { const response = await json(`${API}/action`, {method: 'POST', headers: {'Content-Type': 'application/json', 'X-Token': key}, body: JSON.stringify(body)}, timeout, controller);
      if (!intent || current(intent)) state.lastLinkAckAt = performance.now(); return response; }
    finally { state.controls.delete(controller); }
  }
  function current(intent) { return state.intent === intent && intent.generation === state.generation && !state.hidden; }
  function usableAxis() { return state.online && protocolReady(state.status) && validAxis(state.id) && !state.axisTask && !state.parameterTask; }
  function controls() {
    const enabled = usableAxis();
    for (const profile of Object.keys(PROFILE)) for (const suffix of ['Slider', 'Value']) $(`${profile}${suffix}`).disabled = !enabled;
    $('positionGo').disabled = !enabled;
    $('axisSelect').disabled = !state.online || !protocolReady(state.status) || !!state.axisTask || !!state.parameterTask;
    $('saveNote').disabled = !state.online || !protocolReady(state.status);
    for (const name of ['acquire','zeroSave','signSave','jointSign','readParameters','applyGains']) $(name).disabled = !enabled;
  }
  function sameAxis(value, id) { return value.latest?.selected_id === id; }
  function off(value, id) { const f = value.latest || {}; return sameAxis(value, id) && f.fresh === true
    && f.guard_off_verified === true && f.torque_valid === true && f.torque40_raw === 0 && !f.guard_active; }
  async function chooseAxis(value) {
    if (!state.online || !protocolReady(state.status) || state.parameterTask) { notice('主控E105待安装，组装上电后更新；当前仅显示读数', true); return; }
    const id = axis(Number(value)); if (!validAxis(id)) return;
    cancel('切换关节，先停止旧轴');
    const task = {generation: state.generation, id, profile: 'selection', session: null};
    state.axisTask = task; state.intent = task; controls();
    try {
      const previous = state.status?.latest?.selected_id;
      const probeCancellable = state.status?.latest?.read_only_selection_cancellable === true
        && state.status.latest.guard_active === false;
      if (validAxis(previous) && !probeCancellable) {
        const stopped = await post({op: 'stop'}, task);
        await waitFor(task, v => off(v, previous) && afterEqual(v.latest.completed_request_seq, stopped.request_seq), '旧轴卸力', 5000, stopped.request_seq);
      }
      if (!current(task)) return;
      const selected = await post({op: 'select', id}, task);
      await waitFor(task, v => off(v, id) && v.latest.position_valid === true
        && afterEqual(v.latest.completed_request_seq, selected.request_seq), '所选关节读取', 5000, selected.request_seq);
      if (!current(task)) return;
      state.id = id; state.positionSeeded = false; $('positionValue').value = ''; state.samples = []; state.previous = null;
      state.intent = null; ingest(state.status); notice(`ID${id}已选择；尚未发送运动目标`); log(`已选择ID${id}，只读接管`);
      $('controlState').textContent = `ID${id} · 未启动`;
    } catch (error) { if (current(task)) { state.intent = null; notice(`选轴失败：${error.message}`, true); } }
    finally { if (state.axisTask === task) state.axisTask = null; controls(); }
  }
  function parameterCurrent(task) { return state.parameterTask === task && task.generation === state.generation && task.id === state.id && !state.hidden; }
  async function parameterWait(task, seq, predicate, label) {
    const deadline = performance.now() + 6000;
    while (parameterCurrent(task)) {
      const v = state.status || {}, f = v.latest || {};
      if (sameAxis(v, task.id) && f.error_request_seq === seq && f.error_code) throw new Error(`${label}失败：${f.error_code} / servo_error ${f.servo_error ?? '—'}`);
      if (sameAxis(v, task.id) && predicate(v)) return v;
      if (performance.now() > deadline) throw new Error(label + '回读超时');
      await new Promise(resolve => setTimeout(resolve, 30));
    }
    throw new Error('旧参数操作已取消');
  }
  async function completion(task, reply, field) {
    if (reply.completed === true && reply.id === task.id && reply[field]) return reply[field];
    const v = await parameterWait(task, reply.request_seq, value => {
      const result = value.last_completion; return result?.id === task.id && result.request_seq === reply.request_seq && result.completed === true && !!result[field];
    }, '参数');
    return v.last_completion[field];
  }
  async function parameters(kind) {
    if (!usableAxis()) { notice('请先连接并选择关节', true); return; }
    if (kind === 'apply') cancel('停止并应用XGO位置参数');
    const task = {id: state.id, generation: state.generation, kind}; state.parameterTask = task; controls();
    try {
      if (kind === 'apply') {
        if (!off(state.status, task.id)) {
          const stopped = await post({op: 'stop'});
          await parameterWait(task, stopped.request_seq, v => off(v, task.id) && afterEqual(v.latest.completed_request_seq, stopped.request_seq), '卸力');
        }
        if (!parameterCurrent(task)) return;
        if (!(state.status.latest.mode_valid && state.status.latest.mode33_raw === 4)) {
          const mode = await post({op: 'mode', id: task.id, mode: 4, control_epoch: state.status.control_epoch});
          await parameterWait(task, mode.request_seq, v => off(v, task.id) && v.latest.mode_valid && v.latest.mode33_raw === 4
            && afterEqual(v.latest.completed_request_seq, mode.request_seq), '位置模式');
        }
        if (!parameterCurrent(task)) return;
        $('gainStatus').textContent = `ID${task.id} · RAM参数待回读`;
        const result = await completion(task, await post({op: 'apply_xgo_gains', id: task.id}), 'gain_readback');
        if (!parameterCurrent(task)) return;
        state.gains.set(task.id, {kp: result.actual_kp, kd: result.actual_kd, verified: result.verified === true});
        if (!result.verified) throw new Error('实际增益读回不匹配预期');
        notice(`ID${task.id} XGO位置参数已实读；未启动运动`); log(`ID${task.id} RAM P${result.actual_kp} / D${result.actual_kd} 已读回`);
      } else {
        const result = await completion(task, await post({op: 'read_registers', id: task.id, address: 50, length: 2}), 'register_read');
        if (!parameterCurrent(task)) return;
        if (!result.data_valid || !result.reply_valid || result.actual_address !== 50 || result.actual_length !== 2 || result.data_bytes?.length !== 2) throw new Error('参数回包无有效50/51读数');
        state.gains.set(task.id, {kp: result.data_bytes[0], kd: result.data_bytes[1], verified: false}); log(`ID${task.id} RAM P${result.data_bytes[0]} / D${result.data_bytes[1]} 实读`);
      }
    } catch (error) { if (parameterCurrent(task)) { notice(error.message, true); log(`参数失败：${error.message}`, true); } }
    finally { if (state.parameterTask === task) state.parameterTask = null; renderCalibration(); controls(); }
  }
  function renderCalibration() {
    const id = state.id, zero = state.status?.zero_draft?.joints?.[String(id)], f = state.status?.latest || {};
    const actual = sameAxis(state.status || {}, id) ? sensor(f, 'actual_ticks', 'position_valid', 'position_source_us') : null;
    const angle = jointDegrees(actual, zero);
    $('jointAngle').textContent = angle == null ? '未标定' : `${angle.toFixed(2)}°`;
    $('nativeAngle').textContent = actual == null ? '—' : degrees(actual).toFixed(2);
    $('zeroState').textContent = Number.isFinite(zero?.native_zero_ticks) ? `ID${id} · 草稿p0=${zero.native_zero_ticks}（${degrees(zero.native_zero_ticks).toFixed(2)}°）作为关节0° · 方向${zero.sign === -1 ? '−1' : '+1'}${zero.mode !== 4 ? ' · 坐标模式待复核' : ''} · 模型未标定` : `ID${id ?? '—'} · 软件零位未标定`;
    if (!state.parameterTask) $('jointSign').value = String(state.signDraft.get(id) ?? (zero?.sign === 1 ? 1 : -1));
    const expected = gainExpected(id), gain = state.gains.get(id);
    if (!state.parameterTask) $('gainStatus').textContent = `ID${id ?? '—'} · 当前RAM P${gain?.kp ?? '—'} / D${gain?.kd ?? '—'} · XGO待应用 P${expected.kp} / D${expected.kd}${gain?.verified ? ' · 已核对' : ''}`;
    $('nativeRange').textContent = actual != null && (actual < 0 || actual > 4095) ? '原生位置已超出位置滑条参考0–4095；不夹限反馈、不修改零位' : '位置参考0–4095；软件零位不会移动原生行程边界';
  }
  async function saveZero(capture) {
    if (!usableAxis()) { notice('请先连接并选择关节', true); return; }
    const task = {id: state.id, generation: state.generation}; state.parameterTask = task; controls();
    try {
      const body = {op: capture ? 'zero_save' : 'zero_sign', id: task.id, sign: Number($('jointSign').value)};
      const reply = await post(body);
      if (!parameterCurrent(task)) return;
      if (!reply.saved || !reply.zero_draft || reply.uart_commands_requested !== 0) throw new Error('零位草稿未确认保存');
      state.status.zero_draft = reply.zero_draft; state.signDraft.delete(task.id); notice(capture ? `ID${task.id}当前位置已记录为软件关节0；未写舵机零位` : `ID${task.id}方向草稿已保存，零位保持`); log(capture ? `ID${task.id}保存软件零位草稿` : `ID${task.id}保存方向草稿`);
    } catch (error) { if (parameterCurrent(task)) notice(error.message, true); }
    finally { if (state.parameterTask === task) state.parameterTask = null; renderCalibration(); controls(); }
  }
  function cancel(reason) {
    state.cancelledIntentSinceRead ||= state.intent !== null;
    state.generation++; state.intent = null;
    for (const controller of state.controls) controller.abort(); state.controls.clear();
    for (const controller of state.hbControllers) controller.abort(); state.hbControllers.clear();
    $('controlState').textContent = reason; for (const p of Object.keys(PROFILE)) $(`${p}Slider`).closest('.control')?.classList.remove('active');
  }
  async function stop(reason = '手动停止', expectedEpoch = null) {
    cancel(`${reason}，等待实际卸力`);
    const generation = state.generation;
    const body = {op: 'stop'}; if (Number.isInteger(expectedEpoch)) body.control_epoch = expectedEpoch;
    const request = post(body);
    if (expectedEpoch == null) state.pendingExplicitStop = request;
    try { const result = await request; if (generation === state.generation) log(`${reason}：停止请求已发送 #${result.request_seq ?? '—'}`); return result; }
    catch (error) { if (generation !== state.generation) { log('旧停止回执已过期，不影响当前设定'); return; }
      notice(`停止请求失败：${error.message}；请看扭矩/停机原因实读`, true); log(`停止失败：${error.message}`, true); }
    finally { if (state.pendingExplicitStop === request) state.pendingExplicitStop = null; }
  }
  function haltIntent(intent, reason) {
    if (!current(intent)) return;
    if (Number.isInteger(intent.controlEpoch)) stop(reason, intent.controlEpoch);
    else cancel(`${reason}，旧启动意图已取消`);
  }
  async function waitFor(intent, predicate, description, timeout = 5000, requestSeq = null) {
    const deadline = performance.now() + timeout;
    while (current(intent)) {
      const frame = state.status?.latest || {};
      if (requestSeq != null && frame.error_request_seq === requestSeq && frame.error_code !== 0)
        throw new Error(`${description}失败：error_code=${frame.error_code} / servo_error=${frame.servo_error ?? '—'}`);
      if (predicate(state.status || {})) return state.status;
      if (performance.now() >= deadline) throw new Error(description + '超时，未开始新输出');
      await new Promise(resolve => setTimeout(resolve, 30));
    }
    throw new Error('旧手势已取消');
  }
  function draft(profile, value) {
    value = target(profile, value); $(`${profile}Slider`).value = value; $(`${profile}Value`).value = value;
    const native = percentToNative(profile, value);
    if (profile === 'position') $('positionAngle').textContent = `${degrees(native).toFixed(2)}°`;
    return native;
  }
  async function beginFlow(intent) {
    try {
      const priorStop = state.pendingExplicitStop;
      if (priorStop) {
        const stopped = await priorStop;
        if (!current(intent)) return;
        await waitFor(intent, value => value.latest?.fresh === true && value.latest?.guard_off_verified === true
          && value.latest?.torque40_raw === 0 && afterEqual(value.latest?.completed_request_seq, stopped.request_seq), '手动停止回读');
      }
      const wanted = PROFILE[intent.profile].mode, first = state.status?.latest || {};
      const alreadyOff = first.fresh === true && first.guard_off_verified === true && first.torque_valid === true && first.torque40_raw === 0;
      const alreadyMode = first.mode_valid === true && first.mode33_raw === wanted && first.mode_source_us > 0;
      if (!alreadyOff || !alreadyMode) {
        const stopped = await post({op: 'stop'}, intent);
        if (!current(intent)) return;
        intent.controlEpoch = stopped.control_epoch;
        await waitFor(intent, value => value.latest?.fresh === true && value.latest?.guard_off_verified === true
          && value.latest?.torque_valid === true && value.latest?.torque40_raw === 0
          && afterEqual(value.latest?.completed_request_seq, stopped.request_seq), '卸力回读');
      } else intent.controlEpoch = state.status.control_epoch;
      if (!current(intent)) return;
      if (state.status.latest.guard_anchored !== true || state.status.latest.selected_id !== intent.id) {
        $('controlState').textContent = `首次手势，读取并接管ID${intent.id}`;
        const acquired = await post({op: 'acquire', id: intent.id}, intent);
        await waitFor(intent, value => value.latest?.fresh === true && value.latest?.selected_id === intent.id
          && value.latest?.guard_anchored === true && value.latest?.position_valid === true
          && value.latest?.position_source_us > 0 && value.latest?.guard_off_verified === true
          && afterEqual(value.latest?.completed_request_seq, acquired.request_seq), '首次接管回读');
        if (!current(intent)) return;
      }
      const latest = state.status.latest;
      if (!(latest.mode_valid === true && latest.mode33_raw === wanted && latest.mode_source_us > 0)) {
        $('controlState').textContent = `切换模式${wanted}，等待实读`;
        const changed = await post({op: 'mode', id: intent.id, mode: wanted, control_epoch: state.status.control_epoch}, intent);
        await waitFor(intent, value => value.latest?.fresh === true && value.latest?.mode_valid === true
          && value.latest.mode33_raw === wanted && afterEqual(value.latest.completed_request_seq, changed.request_seq), '模式回读', 5000, changed.request_seq);
        if (!current(intent)) return;
      }
      if (!current(intent)) return;
      const body = {op: 'stream_begin', id: intent.id, control_epoch: state.status.control_epoch, profile: intent.profile, target: intent.target};
      intent.boot = state.status.latest.boot_id; intent.controlEpoch = body.control_epoch; intent.phase = 'starting';
      const response = await post(body, intent);
      if (!current(intent)) return;
      if (response.accepted !== true) throw new Error('Begin未被当前会话接受');
      intent.session = integer(response.session, 1, 0xffffffff, '会话'); intent.beginSeq = response.begin_request_seq ?? response.request_seq;
      intent.controlEpoch = response.control_epoch ?? intent.controlEpoch; intent.phase = 'awaiting';
      $('controlState').textContent = `${intent.profile} 会话${intent.session}，等待当前会话回读`;
      log(`${intent.profile}：目标${nativeToPercent(intent.profile, intent.target).toFixed(4)}% / session ${intent.session}`);
      // Starting-session HB is permitted by Host; old MCU guard/idle frames do not cancel this intent.
      heartbeat();
    } catch (error) { if (current(intent)) { notice(error.message, true); log(error.message, true); haltIntent(intent, '启动未确认'); } }
  }
  function input(profile, value) {
    try {
      if (!usableAxis()) throw new Error('请先连接并选择关节，或等待当前参数操作完成');
      const number = draft(profile, value); let intent = state.intent;
      if (!intent || intent.profile !== profile) {
        cancel('切换控制种类，先卸力');
        intent = {generation: state.generation, id: state.id, profile, target: number, clientSeq: 1, session: null,
          phase: 'switching', slot: null, busy: false, holding: false,
          boot: state.status?.latest?.boot_id, controlEpoch: null};
        state.intent = intent; $(`${profile}Slider`).closest('.control')?.classList.add('active');
        notice(`${profile === 'position' ? '位置模式' : profile === 'speed' ? '速度模式' : '恒流模式'}：设定保持至新值或手动停止`);
        beginFlow(intent);
      } else { intent.target = number; intent.holding = false; intent.clientSeq++; intent.slot = {target: number, client_seq: intent.clientSeq}; }
    } catch (error) { notice(error.message, true); }
  }
  async function pump() {
    const intent = state.intent;
    if (!intent || !current(intent) || intent.session == null || intent.busy || !intent.slot) return;
    const slot = intent.slot; intent.slot = null; intent.busy = true;
    try {
      await post({op: 'stream_update', session: intent.session, client_seq: slot.client_seq, target: slot.target}, intent);
    } catch (error) {
      if (current(intent)) { notice(`更新失败：${error.message}`, true); haltIntent(intent, '更新未确认'); }
    }
    finally { intent.busy = false; }
  }
  function release(profile) {
    const intent = state.intent; if (!intent || intent.profile !== profile) return;
    intent.holding = true; $('controlState').textContent = '设定保持中；新值直接更新，停止按钮卸力';
  }
  async function heartbeat() {
    const intent = state.intent;
    if (!intent || !current(intent) || intent.session == null) return;
    if (performance.now() - state.lastLinkAckAt > 2500 && state.heartbeatFailures > 0) { cancel('控制链路持续失联，旧设定已取消'); return; }
    if (state.hbControllers.size >= 4) return;
    const controller = new AbortController(); state.hbControllers.add(controller);
    try {
      await json(`${API}/action`, {method: 'POST', headers: {'Content-Type': 'application/json', 'X-Token': state.token},
        body: JSON.stringify({op: 'stream_heartbeat', session: intent.session})}, 1800, controller);
      if (current(intent)) { state.lastLinkAckAt = performance.now(); state.heartbeatFailures = 0; }
    } catch (error) {
      if (current(intent) && error.httpStatus === 409) {
        try { const value = await json(`${API}/status`); ingest(value); if (closedMatches(intent, value)) return; } catch {}
      }
      if (current(intent)) {
        if (error.httpStatus === 409) { cancel('当前会话已关闭，旧设定不恢复'); log(`会话关闭：${error.message}`); }
        else { state.heartbeatFailures++; log(`单次续租回执未确认；实际租约由MCU判定：${error.message}`); }
      }
    }
    finally { state.hbControllers.delete(controller); }
  }
  for (const profile of Object.keys(PROFILE)) {
    const slider = $(`${profile}Slider`), field = $(`${profile}Value`);
    slider.addEventListener('pointerdown', event => { state.pointer = profile; slider.setPointerCapture(event.pointerId); });
    slider.addEventListener('input', () => input(profile, slider.value));
    for (const name of ['pointerup', 'pointercancel', 'lostpointercapture']) slider.addEventListener(name, () => { if (state.pointer === profile) { state.pointer = null; release(profile); } });
    slider.addEventListener('keydown', event => { if (['ArrowLeft', 'ArrowRight', 'ArrowUp', 'ArrowDown', 'PageUp', 'PageDown', 'Home', 'End'].includes(event.key)) state.keyHeld = profile; });
    slider.addEventListener('keyup', () => { if (state.keyHeld === profile) { state.keyHeld = null; release(profile); } });
    slider.addEventListener('change', () => { if (state.pointer !== profile && state.keyHeld !== profile) release(profile); });
    field.addEventListener('input', () => {
      try { const percent = target(profile, field.value); draft(profile, percent); if (state.intent?.profile === profile) input(profile, percent); }
      catch (error) { notice(error.message, true); }
    });
  }
  $('positionGo').addEventListener('click', () => { input('position', $('positionValue').value); release('position'); });
  $('positionValue').addEventListener('keydown', event => { if (event.key === 'Enter') { input('position', $('positionValue').value); release('position'); } });
  $('stop').addEventListener('click', () => stop());
  $('axisSelect').addEventListener('change', () => chooseAxis($('axisSelect').value));
  $('acquire').addEventListener('click', async () => { if (!usableAxis()) return; try { const reply = await post({op: 'acquire', id: state.id}); log(`ID${state.id}读取请求 #${reply.request_seq ?? '—'} 已发送`); } catch (error) { notice(error.message, true); } });
  $('jointSign').addEventListener('change', () => state.signDraft.set(state.id, Number($('jointSign').value)));
  $('zeroSave').addEventListener('click', () => saveZero(true)); $('signSave').addEventListener('click', () => saveZero(false));
  $('readParameters').addEventListener('click', () => parameters('read')); $('applyGains').addEventListener('click', () => parameters('apply'));
  $('saveNote').addEventListener('click', async () => { const value = $('annotation').value.trim(); if (!value) return;
    try { await post({op: 'annotation', text: value}); log(`观察：${value}`); $('annotation').value = ''; } catch (error) { notice(error.message, true); } });
  async function exportLog(format) {
    try { const response = await fetch(`${API}/export?format=${format}`, {cache: 'no-store'}); if (!response.ok) throw new Error(`HTTP${response.status}`);
      const url = URL.createObjectURL(await response.blob()), anchor = document.createElement('a'); anchor.href = url; anchor.download = `XGO-assembly-${Date.now()}.${format}`; anchor.click(); setTimeout(() => URL.revokeObjectURL(url), 1000);
    } catch (error) { notice(`导出失败：${error.message}`, true); }
  }
  $('exportCsv').addEventListener('click', () => exportLog('csv')); $('exportJsonl').addEventListener('click', () => exportLog('jsonl'));
  function ingest(value) {
    const latest = value.latest || {}, old = state.status?.latest;
    if (old && old.boot_id === latest.boot_id && Number.isInteger(old.source_seq) && Number.isInteger(latest.source_seq) && !afterEqual(latest.source_seq, old.source_seq)) return;
    if (old && latest.boot_id !== old.boot_id) { state.gains.clear(); cancel('主控重启，旧手势已取消'); }
    state.status = value; state.readAt = performance.now(); state.lastLinkAckAt = performance.now(); state.online = true;
    if (!state.axisTask && validAxis(latest.selected_id) && latest.selected_id !== state.id) {
      state.id = latest.selected_id; state.positionSeeded = false; $('positionValue').value = ''; state.samples = []; state.previous = null;
    }
    if (!state.axisTask) $('axisSelect').value = validAxis(state.id) ? String(state.id) : '';
    $('selectedJoint').textContent = validAxis(state.id) ? `所选关节 ID${state.id}` : '尚未选关节';

    const actual = latest.selected_id === state.id ? sensor(latest, 'actual_ticks', 'position_valid', 'position_source_us') : null;
    const currentRaw = sensor(latest, 'current_raw_signed', 'current_valid', 'current_source_us');
    const mode = sensor(latest, 'mode33_raw', 'mode_valid', 'mode_source_us');
    const torque = sensor(latest, 'torque40_raw', 'torque_valid', 'torque_source_us');
    const echo = latest.goal_echo_valid ? latest.goal_echo || {} : {};
    $('actualPosition').textContent = text(degrees(actual), 2); $('actualCurrent').textContent = text(nativeToPercent('current', currentRaw), 2);
    $('speedFeedbackPercent').textContent = text(nativeToPercent('speed', latest.speed_source_us > 0 ? raw(latest.speed_raw) : null), 4);
    $('positionError').textContent = text(actual != null && Number.isFinite(echo.position) ? degrees(actual - echo.position) : null, 2);
    $('positionEcho').textContent = text(degrees(echo.position), 2); $('speedEcho').textContent = text(nativeToPercent('speed', echo.speed), 4); $('currentEcho').textContent = text(nativeToPercent('current', echo.current), 2);
    $('actualMode').textContent = `当前 ${mode === 4 ? '位置模式' : mode === 1 ? '速度模式' : mode === 2 ? '恒流模式' : '—'}`;
    $('torque').textContent = torque === 0 ? '已卸力' : torque === 1 ? '已上力' : '扭矩状态待确认';
    $('thermal').textContent = latest.voltage_decivolts > 0 && actual != null ? `${latest.temperature_c}℃ / ${(latest.voltage_decivolts / 10).toFixed(1)}V` : '温度 / 电压 —';
    const mcuAge = stamp => Number.isInteger(latest.sample_us) && Number.isInteger(stamp) && stamp > 0 ? ((latest.sample_us - stamp) >>> 0) / 1000 : null;
    $('read65').textContent = latest.status65_valid === true ? `READ65原值 ${latest.status65_raw} · MCU age ${text(mcuAge(latest.status65_source_us), 1)}ms` : 'READ65原值 — · 尚无有效回包';
    $('servoError').textContent = recentReply(latest);
    $('offResult').textContent = `最初device_error ${latest.original_device_error ?? '—'} / guard ${guardText(latest.original_guard_reason)} · OFF错误 ${latest.off_error_code ?? '—'} / 尝试 ${latest.off_attempts ?? '—'}`;
    $('nativeReadings').textContent = `原生位置 ${actual ?? '—'} / 速度 ${latest.speed_raw ?? '—'} / 电流 ${currentRaw ?? '—'}；RAM42/44/46 ${echo.position ?? '—'}/${echo.current ?? '—'}/${echo.speed ?? '—'}；模式${mode ?? '—'}`;
    if (!state.positionSeeded && actual != null && actual >= 0 && actual <= 4095 && String($('positionValue').value).trim() === '') { draft('position', Number(seedPercent('position', actual).toFixed(4))); state.positionSeeded = true; }
    const reason = latest.guard_reason ?? latest.guard_reason_name ?? latest.guard_reason_code ?? '—';
    const guard = `${guardText(reason)} · ${latest.guard_off_verified ? '已确认卸力' : '卸力未确认'}`; $('guard').textContent = `停机原因 ${guard}`;
    if (state.guardKey !== null && state.guardKey !== guard) log(guard, guardIsError(reason, latest.guard_fault)); state.guardKey = guard;
    const stream = value.stream || {};
    $('sequence').textContent = `session ${stream.session ?? '—'} · 接受 ${stream.latest_client_seq ?? '—'} / 应用 ${stream.applied_client_seq ?? '—'} / 舵机回执 ${stream.servo_ack_client_seq ?? '—'}`;
    $('sourceTime').textContent = `MCU源龄：位置 ${text(mcuAge(latest.position_source_us), 1)} / 电流 ${text(mcuAge(latest.current_source_us), 1)}ms · 接收age ${text(latest.age_ms)}ms`;
    const logger = value.logger || {}; $('logger').textContent = `已写 ${logger.written ?? '—'} / 丢弃 ${logger.dropped ?? '—'} / 写错 ${logger.write_errors ?? '—'} / 线程 ${logger.thread_alive === true ? '运行' : '—'}`;
    const intent = state.intent;
    if (intent?.session != null) {
      const epochDelta = Number.isInteger(value.control_epoch) && Number.isInteger(intent.controlEpoch) ? ((value.control_epoch - intent.controlEpoch) >>> 0) : 0;
      if (closedMatches(intent, value)) { log(`会话${intent.session}已结束并确认卸力；${stream.guard_reason ?? reason}`); cancel('当前会话已确认卸力结束'); }
      else if (epochDelta > 0 && epochDelta < 0x80000000) cancel('控制代次改变，旧会话已取消');
      else if (matches(intent, value)) {
        intent.confirmed = true;
        if (latest.guard_fault === true) { notice(`当前会话MCU保护：${reason}`, true); haltIntent(intent, '当前会话保护停止'); }
        else if (latest.guard_off_verified && (latest.guard_completed || ['completed', 'ended', 'stopped', 'fault'].includes(latest.stream_state))) cancel('当前会话已卸力结束');
        else $('controlState').textContent = `${intent.profile} · 设定保持，停止按钮卸力`;
      }
      // While awaiting Begin confirmation, old-session idle/stale/guard frames are display-only.
    }
    $('controllerBuild').textContent = protocolReady(value) ? 'E105 · 主控协议已观察' : '主控E105待安装，组装上电后更新 · 当前只读';
    if (!protocolReady(value) && state.intent) cancel('主控协议未匹配，当前只读');
    renderCalibration(); controls();
    const sourceKey = `${latest.boot_id}:${latest.selected_id}:${latest.source_seq}`;
    if (state.sourceKey !== sourceKey && (actual != null || currentRaw != null)) {
      const time = Number(latest.source_unix_s);
      let speed = Number.isFinite(latest.derived_speed_ticks_s) ? latest.derived_speed_ticks_s : null;
      if (speed == null && state.previous && state.previous.boot === latest.boot_id && time > state.previous.time && time - state.previous.time < 2 && actual != null) speed = (actual - state.previous.actual) / (time - state.previous.time);
      $('derivedSpeed').textContent = text(degrees(speed), 2); $('actualSpeed').textContent = text(degrees(speed), 2);
      state.samples.push({time, actual: degrees(actual), target: degrees(echo.position), current: nativeToPercent('current', currentRaw), boot: latest.boot_id, fresh: latest.fresh !== false}); if (state.samples.length > 500) state.samples.shift();
      state.previous = actual != null ? {time, actual, boot: latest.boot_id} : null; state.sourceKey = sourceKey; draw();
    }
  }
  function drawOne(id, keys, colors) {
    const canvas = $(id), width = canvas.clientWidth || 400, height = canvas.clientHeight || 110, dpr = window.devicePixelRatio || 1;
    canvas.width = width * dpr; canvas.height = height * dpr; const c = canvas.getContext('2d'); c.scale(dpr, dpr); c.clearRect(0, 0, width, height);
    const end = state.samples.at(-1)?.time || Date.now() / 1000, points = state.samples.filter(p => p.time >= end - 30);
    const values = points.flatMap(p => keys.map(k => p[k]).filter(Number.isFinite)); let low = values.length ? Math.min(...values) : 0, high = values.length ? Math.max(...values) : 1;
    const pad = Math.max((high - low) * .15, 1); low -= pad; high += pad; const left = 43, right = width - 8, top = 8, bottom = height - 20;
    c.font = '10px Microsoft YaHei'; c.fillStyle = '#5d7485'; c.strokeStyle = '#d8e4ed';
    for (let i = 0; i < 3; i++) { const y = top + (bottom - top) * i / 2; c.beginPath(); c.moveTo(left, y); c.lineTo(right, y); c.stroke(); c.fillText((high - (high - low) * i / 2).toFixed(0), 1, y + 3); }
    keys.forEach((key, index) => { c.strokeStyle = colors[index]; c.lineWidth = 2; c.beginPath(); let old = null;
      for (const point of points) { if (!point.fresh || !Number.isFinite(point[key])) { old = null; continue; }
        const x = left + (point.time - end + 30) / 30 * (right - left), y = bottom - (point[key] - low) / (high - low) * (bottom - top);
        if (!old || old.boot !== point.boot || point.time - old.time > 1) c.moveTo(x, y); else c.lineTo(x, y); old = point;
      } c.stroke(); });
    c.fillStyle = '#5d7485'; c.fillText('−30s', left, height - 4); c.fillText('现在', right - 25, height - 4);
  }
  function draw() { drawOne('positionPlot', ['target', 'actual'], ['#17344d', '#007e98']); drawOne('currentPlot', ['current'], ['#007e98']); }
  function age() { if (state.readAt && performance.now() - state.readAt > 2500) { state.online = false; controls(); } const latest = state.status?.latest; const milliseconds = state.readAt ? Number(latest?.age_ms || 0) + performance.now() - state.readAt : null;
    $('connection').textContent = milliseconds == null ? '等待读数' : `${state.online && latest?.fresh !== false && milliseconds <= 500 ? '新鲜帧' : '最后值'} · age ${Math.round(milliseconds)}ms`;
  }
  async function poll() {
    try {
      const value = await json(`${API}/status`); ingest(value);
      const failure = state.readFailure;
      if (failure && value.latest?.fresh === true) {
        const cancelledIntent = failure.cancelledIntent || state.cancelledIntentSinceRead;
        const recovered = state.intent ? '读数已恢复，当前设定保持。' : cancelledIntent ? '连接恢复，旧动作未恢复；需要控制时请重新拖动。'
          : '连接恢复，读数已更新；本页未启动动作。';
        if ($('notice').textContent === failure.noticeText) notice(recovered);
        if (!state.intent && $('controlState').textContent === failure.controlText)
          $('controlState').textContent = cancelledIntent ? '连接恢复，旧动作未恢复' : '已连接 · 本页未启动';
        log(recovered); state.readFailure = null;
      }
      if (value.latest?.fresh === true) state.cancelledIntentSinceRead = false;
    }
    catch (error) {
      const message = '主控离线，组装上电后读取；不会自动运行'; notice(message, true);
      if (state.readFailure?.detail !== error.message) log(`读取失败：${error.message}`, true);
      state.readFailure = {cancelledIntent: false, noticeText: message, detail: error.message, controlText: $('controlState').textContent};
    }
    finally { age(); if (!state.hidden) setTimeout(poll, 100); }
  }
  window.addEventListener('resize', draw);
  window.addEventListener('pagehide', () => {
    const intent = state.intent;
    const epoch = intent?.controlEpoch ?? state.status?.control_epoch;
    const body = intent?.session != null ? {op: 'stream_end', session: intent.session, client_seq: intent.clientSeq + 1}
      : Number.isInteger(epoch) ? {op: 'stop', control_epoch: epoch} : null;
    state.hidden = true; cancel('页面关闭，已取消旧手势');
    if (intent && body && state.token) fetch(`${API}/action`, {method: 'POST', keepalive: true,
      headers: {'Content-Type': 'application/json', 'X-Token': state.token}, body: JSON.stringify(body)}).catch(() => {});
  });
  controls(); draw(); token().catch(() => {}); poll(); setInterval(pump, 100); setInterval(heartbeat, 250); setInterval(age, 100);
})();
