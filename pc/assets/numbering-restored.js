/* Official sequential numbering. Opening this page never contacts the robot. */
(() => {
  'use strict';
  const $ = id => document.getElementById(id);
  const IDS = [10,11,12,13,14,20,21,22,23,24,30,31,32,33,34];
  const PENDING_KEY = 'xgoduck-numbering-pending-v1';
  const CANCELLED = new Error('cancelled');
  const slots = new Map();
  let run = null, visible = true, unloading = false, sequence = 0;
  let observed = new Set(), scanned = false, pending = loadPending();
  const note = text => { $('status').textContent = text; };
  function loadPending() {
    try {
      const p = JSON.parse(localStorage.getItem(PENDING_KEY) || 'null');
      return p?.version === 1 && p.source === 1 && IDS.includes(p.target) && typeof p.nonce === 'string' ? p : null;
    } catch { return null; }
  }
  function remember(target) {
    const p = {version:1, source:1, target, nonce:String(Date.now())+'-'+(++sequence)};
    try { localStorage.setItem(PENDING_KEY, JSON.stringify(p)); }
    catch { throw Error('未能保存待核对记录；尚未改号，请允许本页本地存储。'); }
    pending = p;
    return p;
  }
  function forget(p) {
    if (pending?.nonce !== p.nonce) return;
    try { if (loadPending()?.nonce === p.nonce) localStorage.removeItem(PENDING_KEY); }
    catch { throw Error('编号已核对，但待核对记录未能清除；请停止后再继续核对。'); }
    pending = null;
  }
  function paint() {
    const next = scanned ? IDS.find(id => !observed.has(id)) : undefined;
    for (const [id, el] of slots) {
      const state = pending?.target === id ? 'unknown' : observed.has(id) ? 'confirmed' : scanned && id === next ? 'next' : 'unread';
      el.dataset.state = state;
      el.lastChild.textContent = state === 'unknown' ? '待核对' : state === 'confirmed' ? '已读到' : state === 'next' ? '下一颗' : scanned ? '待接入' : '待检查';
    }
    $('target-id').textContent = pending ? pending.target : !scanned ? '待读取' : next ?? '已齐';
    $('target-label').textContent = pending ? '上次改号结果待核对' : '下一颗 ID';
    $('completed-count').textContent = scanned ? observed.size+' / 15' : '— / 15';
    $('restart').disabled = $('continue').disabled = !!run || !visible || unloading;
    $('stop').disabled = !run;
    $('read-state').textContent = scanned ? '本轮真实读取 · '+observed.size+' / 15' : '还没读取总线';
  }
  for (const [group, ids] of [['left',IDS.slice(0,5)],['right',IDS.slice(5,10)],['head',IDS.slice(10)]]) {
    for (const id of ids) {
      const slot = document.createElement('span'); slot.className = 'joint-slot';
      slot.append(document.createTextNode(String(id)), document.createElement('small'));
      $(group+'-slots').append(slot); slots.set(id,slot);
    }
  }
  function phase(id,state) { $(id).dataset.state = state; }
  function stage(r, value, text) {
    if (run !== r || unloading) return;
    document.body.dataset.phase = value;
    if (text) note(text);
  }
  function check(r) { if (r.cancelled || r.unloaded || unloading || run !== r) throw CANCELLED; }
  function pause(r, ms) {
    return new Promise(resolve => {
      r.wake = () => { clearTimeout(r.timer); r.wake = null; resolve(); };
      r.timer = setTimeout(r.wake, ms);
    });
  }
  async function api(path,body) {
    const response = await fetch('/api/'+path, {cache:'no-store', signal:AbortSignal.timeout(4000),
      ...(body === undefined ? {} : {method:'POST', headers:{'Content-Type':'application/json','X-Token':window.NUMBERING_TOKEN},body:JSON.stringify(body)})});
    const value = await response.json();
    if (!response.ok) throw Error(value.detail || '通信未确认');
    return value;
  }
  const servo = (action,id,extra={}) => api('servo',{action,target_id:id,...extra});
  async function ping(id) {
    const value = await servo('ping',id);
    if (value?.target_id !== id || typeof value.ok !== 'boolean') throw Error('编号回复不匹配，已停止。');
    return value.ok;
  }
  const fresh = s => typeof s?.feedback_age_ms === 'number' && Number.isFinite(s.feedback_age_ms) && s.feedback_age_ms >= 0 && s.feedback_age_ms < 500;
  const ready = s => fresh(s) && s.mode === 'servo_debug' && s.mcu_servo_debug === true && s.enabled === false;
  const released = s => fresh(s) && (s.mode === 'shadow' || s.mode === 'off') && s.mcu_servo_debug === false && s.enabled === false;
  async function enter(r) {
    const s = await api('status'); check(r);
    if (s.factory_calibration?.active || s.factory_calibration?.pending || s.mode === 'calibrate') throw Error('正在校准，请结束校准后再编号。');
    if (s.enabled === true || s.mode === 'hold' || s.mode === 'policy') throw Error('先停止运动并卸力，再编号。');
    if (s.mode === 'servo_debug' || s.mcu_servo_debug === true) throw Error('已有维护操作，请先结束后再编号。');
    if (!fresh(s)) throw Error('主控反馈未更新，尚未进入编号。');
    r.entered = true;
    await api('servo/enter',{}); check(r);
    for (let i=0;i<10;i++) {
      const state = await api('status'); check(r);
      if (ready(state)) return;
      await pause(r,100); check(r);
    }
    throw Error('进入维护未确认，已停止编号。');
  }
  async function scan(r) {
    stage(r,'scanning','正在读取已有编号…');
    phase('phase-connect','active');
    const found = new Set(); let source = false;
    for (const id of [1,...IDS]) {
      $('read-state').textContent = '正在读取 ID '+id+'…';
      check(r); const exists = await ping(id); check(r);
      if (id === 1) source = exists;
      else if (exists) found.add(id);
    }
    observed = found; scanned = true; paint();
    phase('phase-connect','done');
    return source;
  }
  // A set_id request is never replayed. Even a failed acknowledgement is
  // followed by independent old/new reads before deciding its outcome.
  async function inspect(p,r) {
    let target, old, read;
    if (r.unloaded || unloading) return 'unknown';
    try { target = await ping(p.target); } catch { /* still read the old ID */ }
    if (!unloading && !r.unloaded) try { old = await ping(1); } catch { /* unknown remains pending */ }
    if (!unloading && !r.unloaded && target === true) try { read = await servo('read',p.target); } catch { /* unknown remains pending */ }
    if (target === true && old === false && read?.target_id === p.target && read.ok === true && typeof read.raw_pos === 'number' && Number.isFinite(read.raw_pos)) return 'changed';
    if (target === false && old === true) return 'unchanged';
    return 'unknown';
  }
  async function resolvePending(r) {
    if (!pending) return;
    const p = pending;
    stage(r,'verifying','先核对上次未确认的改号，不重复写入…');
    const outcome = await inspect(p,r);
    if (unloading || r.unloaded) throw CANCELLED;
    if (outcome === 'unknown') throw Error('上次改号结果仍不明确；请检查只新增了一颗 ID 1，再继续。');
    forget(p); check(r);
  }
  async function number(r,target) {
    check(r);
    const p = remember(target);
    paint(); phase('phase-numbering','active');
    stage(r,'writing','正在把新增 ID 1 编为 '+target+'…');
    let ackFailed = false;
    try { await servo('set_id',1,{new_id:target}); } catch { ackFailed = true; }
    if (unloading || r.unloaded) throw CANCELLED;
    stage(r,'verifying',r.cancelled ? '正在停止；只核对本次改号，不再写下一颗…' : '正在核对新旧编号…');
    phase('phase-centering','active');
    const outcome = await inspect(p,r);
    if (unloading || r.unloaded) throw CANCELLED;
    if (outcome !== 'unknown') forget(p);
    if (outcome === 'unchanged') throw Error('本次没有改成；已停止，不自动重复写入。');
    if (outcome === 'unknown') throw Error('改号结果未确认；已保留待核对记录，不自动重写。');
    check(r); // A late success never paints green in a stopped run.
    observed.add(target); paint();
    for (const id of ['phase-numbering','phase-centering','phase-release']) phase(id,'done');
    note('ID '+target+' 已真实读回'+(ackFailed?'（写入回执未确认，读回已确认）':'')+'；再接下一颗 ID 1。');
  }
  async function exit(r) {
    if (!r.entered || unloading || r.unloaded) return true;
    try { await api('servo/exit',{}); } catch { /* GET decides whether exit succeeded */ }
    for (let i=0;i<10 && !unloading && !r.unloaded;i++) {
      try { if (released(await api('status'))) return true; } catch { /* bounded confirmation only */ }
      if (i<9) await pause(r,100);
    }
    return false;
  }
  async function workflow(r) {
    let message = '', failed = false;
    try {
      stage(r,'entering','正在检查主控并进入编号…');
      await enter(r); await resolvePending(r); check(r);
      let hasSource = await scan(r);
      while (true) {
        check(r);
        const target = IDS.find(id => !observed.has(id));
        if (target === undefined) { message = '15 个标准编号已齐；已有编号未重写。'; break; }
        if (!hasSource) {
          stage(r,'waiting','等你接上一颗出厂 ID 1；下一颗编为 '+target+'。');
          $('connection-hint').textContent = '已编号舵机保留；现在只新增一颗出厂 ID 1。';
          await pause(r,500); check(r);
          hasSource = await ping(1); check(r);
          if (!hasSource) continue;
          hasSource = await scan(r); check(r);
          if (!hasSource) continue;
        }
        const next = IDS.find(id => !observed.has(id));
        if (next === undefined) continue;
        await number(r,next); check(r);
        hasSource = false;
      }
    } catch (error) {
      if (error !== CANCELLED) { failed = true; message = error.message; }
    } finally {
      if (!unloading && !r.unloaded) {
        stage(r,'stopping',r.cancelled?'正在停止并退出维护…':'正在退出维护…');
        const off = await exit(r);
        if (!off) { failed = true; message = '退出维护未确认，请检查主控后再继续。'; }
        if (run === r) {
          run = null;
          document.body.dataset.phase = failed ? 'error' : r.cancelled ? 'idle' : 'complete';
          note(message || '已停止；不会自动继续编号。');
          $('connection-hint').textContent = '已有编号保留；继续编号会重新读取总线。';
          paint();
        }
      }
    }
  }
  function start(restart) {
    if (run || !visible || document.hidden || unloading) return;
    if (restart && !window.confirm('从头检查编号，已有编号保留，不批量重置。每次只新增一颗出厂 ID 1；进入维护会卸力，请支撑好。')) return;
    observed = new Set(); scanned = false;
    for (const id of ['phase-connect','phase-numbering','phase-centering','phase-release']) phase(id,'pending');
    const r = {cancelled:false,entered:false,wake:null,timer:null}; run = r;
    paint(); r.promise = workflow(r);
  }
  function stop() {
    if (!run) return;
    run.cancelled = true; run.wake?.();
    stage(run,'stopping','正在停止；已发出的改号只核对，不重发。');
  }
  $('restart').onclick = () => start(true);
  $('continue').onclick = () => start(false);
  $('stop').onclick = stop;
  window.addEventListener('message', event => {
    if (event.origin !== location.origin || event.source !== parent || event.data?.type !== 'duck-numbering-visibility') return;
    visible = event.data.visible === true;
    if (!visible) stop();
    paint();
  });
  document.addEventListener('visibilitychange', () => { if (document.hidden) stop(); });
  window.addEventListener('pagehide', () => {
    unloading = true;
    if (run) {
      run.cancelled = true; run.unloaded = true; run.wake?.();
      if (run.entered) navigator.sendBeacon('/api/servo/exit',new Blob(['{}'],{type:'application/json'}));
    }
  });
  window.addEventListener('pageshow', () => {
    if (!unloading) return;
    unloading = false; run = null; observed = new Set(); scanned = false; pending = loadPending();
    document.body.dataset.phase = 'idle'; note('已停止；点击继续编号后重新读取，不恢复旧流程。'); paint();
  });
  if (pending) note('有一次改号结果待核对。点击继续编号后先读回，绝不自动重写。');
  paint();
})();
