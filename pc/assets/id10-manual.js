/* ID10 operator probe. No motor POST on page load, polling, or reconnect. */
(function (scope) {
  'use strict';
  const STATUS = '/api/wireless-calibration';
  const ACTION = '/api/wireless-calibration/action';
  const SESSION = '/api/local-session';
  const NATIVE_SHA = '7eb10d544a1db680c9b71638d96693b46015fe85872ddf70a599a6f8ed1b1239';
  const GROUP_SHA = '315a601c81a8f0496d976f1e1f5bac0b85a003bb05d5d8155c552223a853538a';
  const hex64 = value => typeof value === 'string' && /^[0-9a-f]{64}$/.test(value);
  const readable = value => Number.isFinite(value) ? String(value) : '—';

  function assess(envelope) {
    const s = envelope?.status;
    const rows = s?.joints;
    const row = Array.isArray(rows) ? rows.find(item => item?.id === 10) : null;
    const gate = s?.id10_test_once;
    const online = envelope?.connection?.online === true && envelope.connection.update_interlocked !== true;
    const fresh = online && s?.fresh === true && Number.isFinite(s.age_ms) && s.age_ms < 150;
    const allOff = fresh && s.all_torque_off_verified === true && s.torque_may_be_on === false &&
      Array.isArray(rows) && rows.length === 15 && rows.every(item => item?.fresh === true && item.torque_off_verified === true);
    const zero = s?.native_file?.sha256 === NATIVE_SHA && s?.group_file?.sha256 === GROUP_SHA;
    const idle = s?.armed?.active === false && !s?.operation && s?.policy_locked === true;
    const axis = row?.fresh === true && row?.valid === true && row?.mode === 4 &&
      row?.torque_enabled === false && row?.torque_off_verified === true && Number.isInteger(row?.position);
    const contract = gate?.contract_revision === 2 && gate?.available === true && gate?.used === false &&
      gate?.power_mode === 'battery_direct_unlimited' && gate?.runtime_contract_verified === true &&
      gate?.flash_receipt_verified === true && Number.isInteger(gate?.hard_current_raw_cap) &&
      gate.hard_current_raw_cap > 0 && hex64(gate?.mcu_elf_sha256) && hex64(gate?.host_sha256);
    const ready = Boolean(fresh && allOff && zero && idle && axis && contract &&
      Number.isInteger(s.boot_id) && s.boot_id > 0);
    let reason = '单次动作入口就绪；仍须当次确认风险和悬空。';
    if (!online) reason = '板端离线或正在更新；只能尝试停止。';
    else if (!fresh) reason = '板端状态超过 150 ms 或来源不新鲜。';
    else if (!allOff) reason = '15 轴新鲜卸力尚未确认。';
    else if (!zero) reason = '原零位文件身份已变化。';
    else if (!idle) reason = '当前已有授权、动作或策略未锁定。';
    else if (!axis) reason = 'ID10 的 mode4、位置或卸力读回不符合要求。';
    else if (!contract) reason = '动作固件、板端采样保护与刷写回执尚未安装或测试已使用。';
    return {s, row, gate, ready, reason, allOff, fresh};
  }

  function bodyFor(view, delta, confirmations, nonce) {
    if (!view.ready || (delta !== -3 && delta !== 3) || !/^[0-9a-f]{32}$/.test(nonce) ||
        !confirmations || Object.values(confirmations).some(value => value !== true)) return null;
    const s = view.s, gate = view.gate;
    return {op: 'id10_test_once', id: 10, contract_revision: 2,
      expected_boot_id: s.boot_id, expected_mcu_elf_sha256: gate.mcu_elf_sha256,
      expected_host_sha256: gate.host_sha256, expected_native_sha256: NATIVE_SHA,
      expected_group_sha256: GROUP_SHA, delta_ticks: delta, request_nonce: nonce,
      power_mode: 'battery_direct_unlimited', battery_direct_confirmed: true,
      first_feedback_risk_acknowledged: true, hands_clear: true,
      supported: true, leg_suspended: true, clearance_checked: true};
  }

  async function jsonGet(fetcher, path) {
    const response = await fetcher(path, {cache: 'no-store', signal: AbortSignal.timeout(3000)});
    const data = await response.json();
    if (!response.ok) throw Error(data?.error || data?.message || `HTTP ${response.status}`);
    return data;
  }

  async function postOnce(fetcher, body, guard = () => true) {
    // Fetch a current local token for this one click. Never retry a motion POST.
    const session = await jsonGet(fetcher, SESSION);
    if (!/^[0-9a-f]{48}$/.test(session?.token)) throw Error('本地会话无效，本次未发送');
    if (!guard()) throw Error('停止已取消尚未发送的动作');
    const response = await fetcher(ACTION, {method: 'POST', cache: 'no-store',
      headers: {'Content-Type': 'application/json', 'X-Token': session.token},
      body: JSON.stringify(body)});
    const result = await response.json();
    if (!response.ok || result?.ok !== true) throw Error(result?.message || result?.error ||
      result?.detail?.message || result?.code || `HTTP ${response.status}`);
    return result;
  }

  function createController(options) {
    const fetcher = options.fetch;
    let last = null, attempted = false, inFlight = false, stopping = false, stopGeneration = 0;
    async function refresh() {
      const envelope = await jsonGet(fetcher, STATUS);
      last = assess(envelope);
      options.onState?.(last, {attempted, inFlight, stopping});
      return last;
    }
    async function step(delta, confirmations) {
      if (attempted || inFlight || stopping) throw Error('本页仅接受一次动作点击；请先核对结果');
      // Reserve synchronously, before the first await: a second click cannot
      // join the same status refresh, and STOP becomes a cancellation barrier.
      attempted = true; inFlight = true;
      const generation = stopGeneration;
      const maySend = () => !stopping && stopGeneration === generation;
      options.onState?.(last, {attempted, inFlight, stopping});
      try {
        const view = await refresh();
        if (!maySend()) throw Error('停止已取消尚未发送的动作');
        if (!view.ready) throw Error(view.reason);
        const nonce = Array.from(options.randomBytes(16), item => item.toString(16).padStart(2, '0')).join('');
        const body = bodyFor(view, delta, confirmations, nonce);
        if (!body) throw Error('当次确认不完整');
        const result = await postOnce(fetcher, body, maySend);
        if (maySend()) options.onResult?.(result);
        else options.onLateResult?.(result);
        return result;
      } catch (error) {
        if (!maySend()) {
          options.onLateError?.(error);
          return {ok: false, stopped: true};
        }
        throw error;
      } finally {
        inFlight = false;
        try { await refresh(); } catch (error) { options.onError?.(error); }
      }
    }
    async function stop() {
      if (stopping) return null;
      ++stopGeneration;
      attempted = true; // no further movement from this page after a stop
      stopping = true;
      options.onState?.(last, {attempted, inFlight, stopping});
      try {
        // Host latches STOP in memory, sends OFF_ALL immediately, then syncs
        // the stop marker. A later motion POST cannot start after STOP.
        const result = await postOnce(fetcher, {op: 'id10_operator_stop'});
        const view = await refresh();
        if (!view.allOff) throw Error('停止已发送，但 15 轴卸力未确认；请断开动力电源');
        options.onStop?.(result);
        return result;
      } finally { stopping = false; options.onState?.(last, {attempted, inFlight, stopping}); }
    }
    return {refresh, step, stop, getState: () => ({last, attempted, inFlight, stopping})};
  }

  const api = {STATUS, ACTION, SESSION, NATIVE_SHA, GROUP_SHA, assess, bodyFor, createController};
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  scope.ID10Manual = api;

  if (!scope.document) return;
  const get = id => document.getElementById(id);
  const confirmed = () => ({battery_direct_confirmed: get('battery').checked,
    first_feedback_risk_acknowledged: get('risk').checked, hands_clear: get('hands').checked,
    supported: get('support').checked, leg_suspended: get('suspended').checked,
    clearance_checked: get('clearance').checked});
  const render = (view, state) => {
    get('status').textContent = view?.reason || '等待状态';
    get('boot').textContent = readable(view?.s?.boot_id);
    get('position').textContent = readable(view?.row?.position) + ' tick';
    get('current').textContent = readable(view?.row?.current_raw) + ' raw';
    get('current-age').textContent = readable(view?.row?.current_age_ms) + ' ms';
    get('off').textContent = view?.allOff ? '15/15 已卸力' : '未确认';
    get('gate').textContent = view?.gate?.used ? '本次已使用' : view?.gate?.available ? '板端入口可用' : '尚未安装/未放行';
    const ready = view?.ready && !state.attempted && !state.inFlight && !state.stopping &&
      Object.values(confirmed()).every(Boolean);
    get('minus').disabled = !ready;
    get('plus').disabled = !ready;
    get('stop').disabled = false;
  };
  const showError = error => { get('result').textContent = '未确认成功：' + (error?.message || String(error)); };
  const controller = createController({fetch: scope.fetch.bind(scope),
    randomBytes: size => scope.crypto.getRandomValues(new Uint8Array(size)),
    onState: render, onResult: result => {
      get('result').textContent = JSON.stringify({ok: result.ok, delta_ticks: result.delta_ticks,
        all_torque_off_verified: result.all_torque_off_verified,
        evidence_sha256: result.evidence_sha256, dit_trace: result.dit_trace}, null, 2);
    }, onStop: () => { get('result').textContent = '停止已锁存，15/15 轴卸力读回确认。'; },
    onLateResult: () => { get('result').textContent += '\n动作回执晚于停止；以最新 15 轴卸力读回为准。'; },
    onLateError: error => { get('result').textContent += `\n旧动作已取消/拒绝：${error?.message || String(error)}`; },
    onError: showError});
  for (const id of ['battery','risk','hands','support','suspended','clearance'])
    get(id).addEventListener('change', () => render(controller.getState().last, controller.getState()));
  for (const [id, delta] of [['minus', -3], ['plus', 3]])
    get(id).addEventListener('click', event => { if (!event.isTrusted) return;
      void controller.step(delta, confirmed()).catch(showError); });
  get('stop').addEventListener('click', event => { if (!event.isTrusted) return;
    void controller.stop().catch(showError); });
  get('refresh').addEventListener('click', () => void controller.refresh().catch(showError));
  async function batteryLogRefresh() {
    try {
      const log = await jsonGet(scope.fetch.bind(scope), '/api/battery-log');
      const latest = log?.latest || {};
      const supply = log?.latest_observation?.servo_supply_v;
      get('supply').textContent = Number.isFinite(supply) ? `${supply.toFixed(1)} V` : '— V';
      const charge = log?.charging || 'unknown';
      get('battery-log-status').textContent = `记录服务：${log?.running === true ? '运行中' : '未运行'} · 运行状态：${latest.state || latest.operation || '—'} · 充电标注：${charge} · 最近时间：${latest.at_utc || latest.timestamp || '—'}`;
    } catch { get('supply').textContent = '— V'; get('battery-log-status').textContent = '电池记录服务尚未接入或暂不可读'; }
  }
  get('annotate').addEventListener('click', async event => {
    if (!event.isTrusted) return;
    get('annotate').disabled = true;
    try {
      const token = await jsonGet(scope.fetch.bind(scope), SESSION);
      if (!/^[0-9a-f]{48}$/.test(token?.token)) throw Error('本地会话无效');
      const response = await scope.fetch('/api/battery-log/annotation', {method: 'POST',
        headers: {'Content-Type': 'application/json', 'X-Token': token.token},
        body: JSON.stringify({charging: get('charging').value, note: get('charge-note').value.slice(0,160)})});
      const data = await response.json();
      if (!response.ok) throw Error(data?.error || data?.message || `HTTP ${response.status}`);
      get('battery-log-status').textContent = `充电状态已记录：${get('charging').selectedOptions[0].textContent}`;
    } catch (error) { get('battery-log-status').textContent = `充电标注未确认：${error.message}`; }
    finally { get('annotate').disabled = false; }
  });
  void controller.refresh().catch(showError);
  void batteryLogRefresh();
  setInterval(() => { if (!controller.getState().inFlight) void controller.refresh().catch(showError); }, 1000);
  setInterval(() => void batteryLogRefresh(), 10000);
})(typeof window !== 'undefined' ? window : globalThis);
