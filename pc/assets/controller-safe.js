(function () {
  'use strict';
  const TICKS_PER_TURN = 4096;
  const MODES = {0: '0 · 位置', 1: '1 · 速度', 2: '2 · 电流', 4: '4 · PD'};
  function signedInteger(value, label = '数值') {
    const number = typeof value === 'string' && value.trim() === '' ? NaN : Number(value);
    if (!Number.isInteger(number) || number < -32767 || number > 32767) throw new Error(`${label}须为 −32767 至 32767 的整数`);
    return number;
  }
  function angleToPosition(origin, angle) {
    if (!Number.isInteger(origin)) throw new Error('本次起点尚未回读，请先接管并记录起点');
    if (!Number.isFinite(Number(angle)) || String(angle).trim() === '') throw new Error('相对角度无效');
    return signedInteger(origin + Math.round(Number(angle) * TICKS_PER_TURN / 360), '位置');
  }
  function positionToAngle(origin, position) { return Number.isInteger(origin) ? (position - origin) * 360 / TICKS_PER_TURN : null; }
  function signedRaw(value) {
    if (!Number.isInteger(value)) return null;
    if (value < 0) return value;
    if (value === 0x8000) return 0; // CRC-valid sign-magnitude negative zero is a successful zero reading.
    return value & 0x8000 ? -(value & 0x7fff) : value;
  }
  function derivedSpeed(previous, current) {
    if (!previous || previous.boot !== current.boot || current.time <= previous.time || current.time - previous.time > 2) return null;
    return (current.actual - previous.actual) / (current.time - previous.time);
  }
  function goalEcho(latest) {
    return {position: latest.goal_echo?.position ?? latest.raw42_echo_position ?? latest.target_ticks,
      speed: latest.goal_echo?.speed ?? latest.raw46_echo_speed ?? latest.target_speed_raw,
      current: latest.goal_echo?.current ?? latest.raw44_echo_current ?? latest.target_current_raw};
  }
  function validateSafeSettings(body, mode, anchor) {
    for (const field of ['position', 'speed', 'current']) signedInteger(body[field], field);
    if (![0, 1, 2, 4].includes(mode)) throw new Error('先回读并确认控制模式0/1/2/4');
    if (!Number.isInteger(anchor)) throw new Error('先接管，建立固定起点与±60°围栏');
    if (Math.abs(body.position - anchor) > 683) throw new Error('目标超出固定起点±60°围栏（±683 ticks）');
    if (Math.abs(body.speed) > 228) throw new Error('速度设定超过228上限（位置约20°/s）');
    if (mode === 2) {
      if (Math.abs(body.current) > 64) throw new Error('恒流模式上限为±64 raw，单次500ms并冷却2秒');
    } else if (body.current < 0 || body.current > 128) throw new Error('此模式电流限制须为0至128 raw；0明确使用默认64');
    if ((mode === 0 || mode === 4) && body.speed < 0) throw new Error('位置目标斜坡须为0至228 ticks/s；0明确使用默认114');
    return body;
  }
  function deadlineSeconds(body, mode, actual) {
    if (mode === 1) return 1;
    if (mode === 2) return .5;
    return Math.min(12, Math.abs(body.position - actual) / (body.speed || 114) + 1.5);
  }
  function requireIdle(latest, op) {
    if (latest?.guard_active === true) throw new Error('任务正在运行，请先停止并等卸力确认，再发新设定');
    if (op === 'acquire') return;
    if (!latest || latest.guard_off_verified !== true) throw new Error('先停止，并等待实际卸力确认');
    const phase = latest.guard_phase;
    if (!(phase === 0 || phase === 'idle' || phase === 'idle_off')) throw new Error('等待控制器回到空闲状态，再发新设定');
  }
  function sameLeaseTask(intent, latest) {
    return Number.isInteger(intent?.runStarted) && latest?.boot_id === intent.boot && latest.run_started_us === intent.runStarted;
  }
  function requirePositionMode(mode) {
    if (mode !== 0 && mode !== 4) throw new Error('±45°和回起点仅用于位置模式0/4；当前模式忽略位置，请使用“执行设定”明确发对应脉冲');
  }
  function readable(latest, flag, stamp) {
    return latest?.[flag] === true && Number.isInteger(latest[stamp]) && latest[stamp] > 0;
  }
  function sensorValues(latest) {
    const position = readable(latest, 'position_valid', 'position_source_us');
    const current = readable(latest, 'current_valid', 'current_source_us');
    const mode = readable(latest, 'mode_valid', 'mode_source_us');
    const torque = readable(latest, 'torque_valid', 'torque_source_us');
    const echo = latest.goal_echo_valid === true ? goalEcho(latest) : {position: null, speed: null, current: null};
    const thermal = position && current && Number.isFinite(latest.voltage_decivolts) && latest.voltage_decivolts > 0;
    return {actual: position ? latest.actual_ticks : null,
      current: current ? latest.current_raw_signed ?? signedRaw(latest.current_raw) : null,
      speed: Number.isInteger(latest.speed_source_us) && latest.speed_source_us > 0 ? signedRaw(latest.speed_raw) : null,
      mode: mode ? latest.mode33_raw : null, torque: torque ? latest.torque40_raw : null, echo,
      temperature: thermal ? latest.temperature_c : null, voltage: thermal ? latest.voltage_decivolts / 10 : null};
  }
  const helpers = {signedInteger, angleToPosition, positionToAngle, signedRaw, derivedSpeed, goalEcho, validateSafeSettings, deadlineSeconds, requireIdle, sameLeaseTask, requirePositionMode, readable, sensorValues};
  if (typeof window === 'undefined') { if (typeof module !== 'undefined') module.exports = helpers; return; }
  const $ = id => document.getElementById(id);
  const state = {token: null, origin: null, latest: null, readAt: 0, ageAtRead: 0, fresh: false,
    connected: false, touched: false, modeTouched: false, cursor: 0, sourceKey: null, previous: null,
    points: [], pointKeys: new Set(), pending: new Map(), polling: true,
    leaseIntent: null, leaseInFlight: false, leaseAbort: null, operationPending: null, modeActual: null, lastGuardKey: null,
    commandEpoch: 0, controlAbort: null};
  const numberText = (value, decimals = 0) => Number.isFinite(value) ? value.toFixed(decimals) : '—';
  function notice(text, error = false) { $('notice').textContent = text; $('notice').classList.toggle('error', error); }
  function log(text, error = false, at = Date.now()) {
    const row = document.createElement('div'); row.className = `event-row${error ? ' error' : ''}`;
    const time = document.createElement('time'); time.textContent = new Date(at).toLocaleTimeString('zh-CN', {hour12: false});
    const body = document.createElement('span'); body.textContent = text; row.append(time, body);
    $('log').prepend(row); while ($('log').children.length > 120) $('log').lastChild.remove();
  }
  async function request(path, options = {}, suppliedController = null) {
    const controller = suppliedController || new AbortController(); const timer = setTimeout(() => controller.abort(), 9000);
    try {
      const reply = await fetch(path, {cache: 'no-store', credentials: 'same-origin', ...options, signal: controller.signal});
      const raw = await reply.text(); let data;
      try { data = JSON.parse(raw); } catch { throw new Error(`HTTP ${reply.status}：${raw.slice(0, 300)}`); }
      if (!reply.ok || data.ok === false || data.sent === false && data.error) {
        const error = new Error(`HTTP ${reply.status}：${data.error || data.message || raw.slice(0, 500)}`);
        error.status = reply.status; error.data = data; throw error;
      }
      return data;
    } finally { clearTimeout(timer); }
  }
  async function refreshToken() { state.token = (await request('/api/local-session')).token; }
  function stopLease(reason = '未续租') {
    state.leaseIntent = null; if (state.leaseAbort) state.leaseAbort.abort();
    $('leaseState').textContent = reason;
  }
  function cancelPendingControl(reason) {
    state.commandEpoch++; stopLease(reason);
    if (state.controlAbort) state.controlAbort.abort();
    state.operationPending = null; state.pending.clear();
  }
  async function heartbeat() {
    const intent = state.leaseIntent; const latest = state.latest;
    if (!intent || state.leaseInFlight) return;
    if (!state.polling || !state.connected || !state.fresh || !latest || latest.boot_id !== intent.boot
        || performance.now() - state.readAt > 1000 || latest.guard_fault === true
        || latest.position_age_ms > 150 || latest.current_age_ms > 150) {
      stopLease('源失鲜 / 断开，已停止续租'); return;
    }
    if (latest.guard_active !== true) return;
    if (!intent.activeSeen) return;
    if (!sameLeaseTask(intent, latest)) { stopLease('任务身份改变，已停止续租'); return; }
    if (!state.token) { stopLease('会话失效，已停止续租'); return; }
    state.leaseInFlight = true; state.leaseAbort = new AbortController();
    try {
      await request('/api/controller-safe/action', {method: 'POST', headers: {'Content-Type': 'application/json', 'X-Token': state.token},
        body: JSON.stringify({op: 'heartbeat'})}, state.leaseAbort);
      if (state.leaseIntent === intent) { intent.activeSeen = true; $('leaseState').textContent = '本页续租中 · 250ms'; }
    } catch (error) {
      if (state.leaseIntent === intent) { stopLease('续租失败，已停止'); notice(`续租失败：${error.message}；不自动恢复`, true); log(`续租失败：${error.message}`, true); }
    } finally { state.leaseInFlight = false; state.leaseAbort = null; }
  }
  async function action(body, label) {
    const submittedEpoch = state.commandEpoch; let controlAbort = null;
    try {
      const startsOrChanges = body.op === 'apply' || body.op === 'mode' || body.op === 'acquire' || body.op === 'release' || body.op === 'torque' && body.enabled === true;
      if (startsOrChanges) {
        if (state.operationPending !== null) throw new Error('上一请求正在处理，请先等待；停止按钮始终可用');
        requireIdle(state.latest, body.op);
        state.operationPending = 'sending';
      }
      if (!state.token) await refreshToken();
      if (startsOrChanges && submittedEpoch !== state.commandEpoch) throw new Error('该操作已被停止或页面状态取消，本次未发送');
      if (startsOrChanges) { controlAbort = new AbortController(); state.controlAbort = controlAbort; }
      const result = await request('/api/controller-safe/action', {method: 'POST',
        headers: {'Content-Type': 'application/json', 'X-Token': state.token}, body: JSON.stringify(body)}, controlAbort);
      if (startsOrChanges && submittedEpoch !== state.commandEpoch) {
        log(`${label}：收到旧请求回执；不恢复续租，实际输出以保护状态为准`); return result;
      }
      const seq = result.request_seq;
      if (startsOrChanges) state.operationPending = Number.isInteger(seq) ? seq : null;
      if ((body.op === 'apply' || body.op === 'torque' && body.enabled === true) && result.sent === true
          && state.connected && state.fresh && Number.isInteger(state.latest?.boot_id)) {
        state.leaseIntent = {boot: state.latest.boot_id, seq, activeSeen: false, runStarted: null};
        $('leaseState').textContent = '本次明确操作，等待保护任务启动';
      }
      if (Number.isInteger(seq)) state.pending.set(seq, label);
      const text = result.acked === true || result.completed === true ? `${label}：请求已确认${seq == null ? '' : ` #${seq}`}`
        : result.sent === true ? `${label}：已发送${seq == null ? '' : ` #${seq}`}，等待设备回读` : `${label}：${JSON.stringify(result)}`;
      notice(text); log(text); return result;
    } catch (error) {
      if (submittedEpoch !== state.commandEpoch) { log(`${label}：旧请求已取消，不恢复续租`); return null; }
      state.operationPending = null;
      notice(`${label}失败：${error.message}`, true); log(`${label}失败：${error.message}`, true);
      if (error.status === 403) { state.token = null; refreshToken().catch(() => {}); }
      return null; // Never retry or queue a failed command, including after reconnection.
    } finally { if (state.controlAbort === controlAbort) state.controlAbort = null; }
  }
  function setPosition(value, edited = false) {
    value = signedInteger(value, '位置'); $('position').value = value; $('positionRange').value = value;
    $('dialValue').textContent = value; $('dial').setAttribute('aria-valuenow', value);
    $('dialNeedle').style.transform = `rotate(${value * 360 / TICKS_PER_TURN}deg)`;
    const relative = positionToAngle(state.origin, value); if (relative !== null) $('angle').value = relative.toFixed(2);
    if (edited) state.touched = true;
  }
  function settings() { return validateSafeSettings({op: 'apply', position: signedInteger($('position').value, '位置'),
    speed: signedInteger($('speed').value, '速度设定'), current: signedInteger($('current').value, '电流原值')}, state.modeActual, state.origin); }
  function updateBudget() {
    if (!Number.isInteger(state.origin) || String($('position').value).trim() === '' || !Number.isFinite(sensorValues(state.latest || {}).actual)) {
      $('taskBudget').textContent = '等待接管并读到固定起点；当前位置尚未确认。'; return;
    }
    try { const body = settings(); const mode = state.modeActual;
      const seconds = deadlineSeconds(body, mode, state.latest.actual_ticks);
      $('taskBudget').textContent = mode === 2 ? '恒流≤±64 raw · 500ms硬截止 · 结束后冷却2秒'
        : mode === 1 ? '速度≤±228原值 · 单次1秒硬截止'
          : `目标斜坡 ${body.speed || 114} ticks/s · ${(seconds).toFixed(2)}秒硬截止（最长12秒）；0速度=114，0电流=64`;
    } catch (error) { $('taskBudget').textContent = error.message; }
  }
  function executePosition(value, label) { try { requirePositionMode(state.modeActual); setPosition(value, true); action(settings(), label); } catch (error) { notice(error.message, true); } }
  $('position').addEventListener('input', () => { state.touched = true; try { setPosition($('position').value, true); } catch {} });
  $('positionRange').addEventListener('input', event => setPosition(Number(event.target.value), true));
  $('angle').addEventListener('change', () => { try { setPosition(angleToPosition(state.origin, $('angle').value), true); } catch (error) { notice(error.message, true); } });
  for (const id of ['position', 'angle', 'speed', 'current']) $(id).addEventListener('change', updateBudget);
  for (const id of ['speed', 'current']) $(id).addEventListener('input', () => { state.touched = true; updateBudget(); });
  let dragging = false;
  function dialPointer(event) {
    if (!dragging || ![0, 4].includes(state.modeActual)) return;
    const box = $('dial').getBoundingClientRect();
    const radians = Math.atan2(event.clientX - (box.left + box.width / 2), -(event.clientY - (box.top + box.height / 2)));
    const phase = ((radians / (2 * Math.PI) + 1) % 1) * TICKS_PER_TURN;
    const previous = Number($('position').value) || 0;
    const selected = Math.round(phase + Math.round((previous - phase) / TICKS_PER_TURN) * TICKS_PER_TURN);
    if (selected >= -32767 && selected <= 32767) setPosition(selected, true);
  }
  $('dial').addEventListener('pointerdown', event => { if (![0, 4].includes(state.modeActual)) return; dragging = true; $('dial').setPointerCapture(event.pointerId); dialPointer(event); });
  $('dial').addEventListener('pointermove', dialPointer);
  for (const type of ['pointerup', 'pointercancel']) $('dial').addEventListener(type, () => { dragging = false; });
  $('dial').addEventListener('keydown', event => {
    if (![0, 4].includes(state.modeActual)) return;
    const step = {ArrowRight: 1, ArrowUp: 1, ArrowLeft: -1, ArrowDown: -1}[event.key];
    if (!step) return; event.preventDefault(); try { setPosition(Number($('position').value) + step, true); } catch {}
  });
  $('apply').addEventListener('click', () => { try { action(settings(), '执行设定'); } catch (error) { notice(error.message, true); } });
  $('torqueOn').addEventListener('click', () => action({op: 'torque', enabled: true}, '上力'));
  $('stop').addEventListener('click', () => { cancelPendingControl('手动停止，已停止续租'); action({op: 'stop'}, '停止 / 卸力'); });
  $('applyMode').addEventListener('click', () => { cancelPendingControl('应用模式，已停止旧任务续租'); action({op: 'mode', mode: Number($('mode').value)}, '应用模式'); });
  $('mode').addEventListener('change', () => { state.modeTouched = true; });
  $('acquire').addEventListener('click', () => action({op: 'acquire'}, '接管并记录起点'));
  $('release').addEventListener('click', () => { cancelPendingControl('释放控制，已停止续租'); action({op: 'release'}, '释放控制'); });
  $('plus45').addEventListener('click', () => { try { executePosition(angleToPosition(state.origin, 45), '+45°'); } catch (error) { notice(error.message, true); } });
  $('minus45').addEventListener('click', () => { try { executePosition(angleToPosition(state.origin, -45), '−45°'); } catch (error) { notice(error.message, true); } });
  $('home').addEventListener('click', () => { if (Number.isInteger(state.origin)) executePosition(state.origin, '回本次起点'); else notice('本次起点尚未回读，请先接管并记录起点', true); });
  $('saveNote').addEventListener('click', async () => { const text = $('annotation').value.trim(); if (!text) return;
    if (await action({op: 'annotation', text}, '记录观察')) $('annotation').value = ''; });
  async function exportFile(format) {
    try {
      const reply = await fetch(`/api/controller-safe/export?format=${format}`, {cache: 'no-store', credentials: 'same-origin'});
      if (!reply.ok) throw new Error(`HTTP ${reply.status}：${(await reply.text()).slice(0, 300)}`);
      const url = URL.createObjectURL(await reply.blob()); const anchor = document.createElement('a'); anchor.href = url;
      anchor.download = `ID10-${new Date().toISOString().replace(/[:.]/g, '-')}.${format}`; anchor.click(); setTimeout(() => URL.revokeObjectURL(url), 1000);
    } catch (error) { notice(`导出失败：${error.message}`, true); }
  }
  $('exportCsv').addEventListener('click', () => exportFile('csv'));
  $('exportJsonl').addEventListener('click', () => exportFile('jsonl'));
  function appendSample(latest, fresh) {
    const sensors = sensorValues(latest);
    const sourceKey = `${latest.boot_id}:${latest.source_seq ?? latest.source_unix_s}`;
    if (state.pointKeys.has(sourceKey)) return;
    const time = Number(latest.source_unix_s);
    if (!Number.isFinite(time) || (!Number.isFinite(sensors.actual) && !Number.isFinite(sensors.current))) return;
    state.pointKeys.add(sourceKey);
    state.points.push({key: sourceKey, time, positionTime: Number(latest.position_source_unix_s) || time,
      currentTime: Number(latest.current_source_unix_s) || time,
      actual: sensors.actual, target: sensors.echo.position,
      current: sensors.current, boot: latest.boot_id, fresh,
      positionFresh: fresh && !(latest.position_age_ms > 500), currentFresh: fresh && !(latest.current_age_ms > 500)});
    state.points.sort((left, right) => left.time - right.time);
    if (state.points.length > 1800) { state.points.splice(0, state.points.length - 1800);
      state.pointKeys = new Set(state.points.map(point => point.key)); }
  }
  function ingest(value) {
    const latest = value.latest; if (!latest) return;
    if (state.latest && latest.boot_id !== state.latest.boot_id) {
      cancelPendingControl('主控重启，已停止续租'); state.origin = null;
    }
    state.latest = latest; state.readAt = performance.now();
    state.ageAtRead = Math.max(Number(latest.age_ms ?? value.age_ms) || 0,
      Number(latest.position_age_ms) || 0, Number(latest.current_age_ms) || 0);
    state.connected = true; state.fresh = (latest.fresh ?? value.fresh) === true;
    const origin = latest.guard_anchored === true ? latest.anchor_ticks : value.origin_ticks ?? latest.origin_ticks;
    if (Number.isInteger(origin)) { state.origin = origin; $('origin').textContent = origin; }
    const sensors = sensorValues(latest); const actual = sensors.actual; const echo = sensors.echo; const target = echo.position;
    if (!state.touched && Number.isInteger(actual)) {
      if (Number.isInteger(state.origin)) setPosition(angleToPosition(state.origin, 45));
    }
    const current = sensors.current;
    const time = Number(latest.position_source_unix_s ?? latest.source_unix_s); const point = {time, actual, target,
      current, boot: latest.boot_id, fresh: state.fresh};
    const derived = Number.isFinite(actual) ? Number.isFinite(latest.derived_speed_ticks_s) ? latest.derived_speed_ticks_s : derivedSpeed(state.previous, point) : null;
    $('actual').textContent = numberText(actual); $('target').textContent = numberText(target);
    $('derivedSpeed').textContent = numberText(derived, 1); $('rawSpeed').textContent = numberText(sensors.speed);
    $('rawCurrent').textContent = numberText(current);
    $('thermal').textContent = sensors.temperature === null && sensors.voltage === null ? '—' : `${numberText(sensors.temperature)}° / ${numberText(sensors.voltage, 1)}V`;
    const mode = sensors.mode; state.modeActual = mode; $('modeActual').textContent = MODES[mode] || (mode == null ? '—' : `${mode} · 实读`);
    const positionMode = mode === 0 || mode === 4;
    for (const id of ['plus45', 'minus45', 'home', 'position', 'positionRange', 'angle']) $(id).disabled = !positionMode;
    $('dial').setAttribute('aria-disabled', String(!positionMode));
    $('dial').tabIndex = positionMode ? 0 : -1;
    $('dial').classList.toggle('mode-ignored', !positionMode);
    $('modeSemantics').textContent = mode === 2 ? '恒流模式：忽略位置和速度字段；执行设定发500ms电流脉冲。'
      : mode === 1 ? '速度模式：忽略位置字段；执行设定发1秒速度脉冲。'
        : positionMode ? '位置模式：速度为本地目标斜坡，电流为限制；按实际回读检查响应。' : '等待实际模式回读；位置快捷动作未启用。';
    if (!state.modeTouched && Object.hasOwn(MODES, mode)) $('mode').value = mode;
    $('torqueActual').textContent = sensors.torque === 0 ? '0 · 卸力' : sensors.torque === 1 ? '1 · 上力' : numberText(sensors.torque);
    $('speedEcho').textContent = numberText(signedRaw(echo.speed));
    $('currentEcho').textContent = numberText(signedRaw(echo.current));
    $('speedLabel').textContent = mode === 0 || mode === 4 ? '位置斜坡 · ticks/s' : '速度设定 · signed raw';
    const reason = latest.guard_reason ?? latest.guard_reason_name ?? latest.guard_reason_code ?? '—';
    $('guardResult').textContent = `保护 ${reason} · ${latest.guard_active ? '任务运行' : '任务停止'} · ${latest.guard_off_verified ? '已回读卸力' : '卸力未确认'}${latest.guard_cooldown ? ' · 冷却中' : ''}`;
    const guardKey = `${latest.boot_id}:${reason}:${latest.guard_active}:${latest.guard_off_verified}:${latest.guard_cooldown}`;
    if (state.lastGuardKey !== null && guardKey !== state.lastGuardKey) log($('guardResult').textContent, latest.guard_fault === true);
    state.lastGuardKey = guardKey;
    if (state.leaseIntent) {
      const intent = state.leaseIntent;
      if (!state.fresh || latest.boot_id !== intent.boot || latest.guard_fault === true
          || latest.error_request_seq === intent.seq
          || latest.guard_completed === true && (intent.activeSeen || latest.completed_request_seq === intent.seq)
          || latest.guard_off_verified === true && intent.activeSeen) {
        stopLease('任务结束 / 保护停止，已停止续租');
      } else if (intent.activeSeen && !sameLeaseTask(intent, latest)) {
        stopLease('任务身份改变，已停止续租');
      } else if (latest.guard_active === true && !intent.activeSeen
          && (latest.ack_request_seq === intent.seq || latest.completed_request_seq === intent.seq)
          && Number.isInteger(latest.run_started_us)) {
        intent.runStarted = latest.run_started_us; intent.activeSeen = true;
      }
    }
    updateBudget();
    const sourceKey = `${latest.boot_id}:${latest.source_seq ?? latest.source_unix_s}`;
    if (sourceKey !== state.sourceKey && Number.isFinite(time) && (Number.isFinite(actual) || Number.isFinite(current))) {
      appendSample(latest, state.fresh);
      state.sourceKey = sourceKey; state.previous = Number.isFinite(actual) ? point : null; drawPlots();
    }
    const logState = value.logger || {};
    $('loggerState').textContent = `位置 age ${numberText(latest.position_age_ms)} ms　电流 age ${numberText(latest.current_age_ms)} ms　日志已写 ${logState.written ?? '—'} / 丢弃 ${logState.dropped ?? logState.drops ?? '—'} / 写错 ${logState.write_errors ?? '—'} / 线程 ${logState.thread_alive === true ? '运行' : logState.thread_alive === false ? '停止' : '—'}${logState.error ? `　${logState.error}` : ''}`;
    for (const [seq, label] of state.pending) {
      const failed = (value.error_request_seq ?? latest.error_request_seq) === seq;
      const completed = (value.completed_request_seq ?? latest.completed_request_seq) === seq;
      if (failed || completed) {
        const error = value.servo_error ?? latest.servo_error;
        const message = failed ? `${label} #${seq}：请求失败 ${error ?? ''}` : `${label} #${seq}：请求已处理`;
        notice(message, failed); log(message, failed); state.pending.delete(seq);
        if (state.operationPending === seq) state.operationPending = null;
      }
    }
  }
  function drawPlot(canvas, keys, colors, minTime, maxTime) {
    const width = Math.max(200, canvas.clientWidth); const height = canvas.clientHeight || 120;
    const dpr = window.devicePixelRatio || 1; canvas.width = Math.round(width * dpr); canvas.height = Math.round(height * dpr);
    const context = canvas.getContext('2d'); context.scale(dpr, dpr); context.clearRect(0, 0, width, height);
    const left = 43, right = width - 9, top = 9, bottom = height - 21;
    const points = state.points.filter(point => point.time >= minTime && point.time <= maxTime);
    const values = points.flatMap(point => point.fresh ? keys.map(key => point[key]).filter(Number.isFinite) : []);
    let low = values.length ? Math.min(...values) : 0; let high = values.length ? Math.max(...values) : 1;
    const pad = Math.max((high - low) * .15, 1); low -= pad; high += pad;
    context.font = '10px Microsoft YaHei'; context.textAlign = 'right'; context.fillStyle = '#597082'; context.strokeStyle = '#dce7ef'; context.lineWidth = 1;
    for (let index = 0; index <= 2; index++) { const y = top + (bottom - top) * index / 2;
      context.beginPath(); context.moveTo(left, y); context.lineTo(right, y); context.stroke(); context.fillText((high - (high - low) * index / 2).toFixed(0), left - 6, y + 3); }
    context.textAlign = 'center'; for (let index = 0; index <= 3; index++) {
      context.fillText(`${Math.round((maxTime - minTime) * (index / 3 - 1))}s`, left + (right - left) * index / 3, height - 5); }
    keys.forEach((key, index) => { context.strokeStyle = colors[index]; context.lineWidth = index === 0 && keys.length > 1 ? 1.5 : 2;
      context.setLineDash(index === 0 && keys.length > 1 ? [4, 3] : []); context.beginPath(); let previous = null;
      let previousTime = null;
      const sampleTime = point => key === 'current' ? point.currentTime : point.positionTime;
      const ordered = points.slice().sort((a, b) => sampleTime(a) - sampleTime(b));
      for (const point of ordered) { if (!point.fresh || !(key === 'current' ? point.currentFresh : point.positionFresh) || !Number.isFinite(point[key])) { previous = null; continue; }
        const sourceTime = sampleTime(point);
        const x = left + (sourceTime - minTime) / Math.max(.01, maxTime - minTime) * (right - left);
        const y = bottom - (point[key] - low) / (high - low) * (bottom - top);
        if (!previous || sourceTime - previousTime > 1 || point.boot !== previous.boot) context.moveTo(x, y); else context.lineTo(x, y);
        previous = point; previousTime = sourceTime;
      } context.stroke(); context.setLineDash([]);
    });
    if (!values.length) { context.fillStyle = '#7e94a4'; context.textAlign = 'center'; context.fillText('等待同源采样', (left + right) / 2, (top + bottom) / 2); }
  }
  function drawPlots() {
    const last = state.points[state.points.length - 1]; const end = last ? last.time : Date.now() / 1000;
    drawPlot($('positionPlot'), ['target', 'actual'], ['#17344d', '#007e98'], end - 30, end);
    drawPlot($('currentPlot'), ['current'], ['#007e98'], end - 30, end);
  }
  function updateAge() {
    const age = state.readAt ? state.ageAtRead + performance.now() - state.readAt : null;
    const live = state.connected && state.fresh && age !== null && age <= 500;
    $('connection').classList.toggle('offline', !live);
    $('connection').textContent = age === null ? '等待连接' : `${live ? '实时读数' : '最后值'} · age ${Math.round(age)} ms`;
  }
  async function pollStatus() {
    try { ingest(await request('/api/controller-safe/status')); }
    catch (error) { state.connected = false; state.fresh = false; cancelPendingControl('读取失败，已停止续租'); $('loggerState').textContent = `读取失败：${error.message}；保留最后值`; }
    finally { updateAge(); if (state.polling) setTimeout(pollStatus, 100); }
  }
  window.addEventListener('resize', drawPlots);
  window.addEventListener('pagehide', () => { state.polling = false; cancelPendingControl('页面关闭，已停止续租'); });
  drawPlots(); updateBudget(); refreshToken().catch(() => {}); pollStatus(); setInterval(updateAge, 100); setInterval(heartbeat, 250);
})();
