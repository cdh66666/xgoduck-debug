/* This view observes feedback only. No motion, calibration or deployment writes. */
(() => {
  'use strict';
  const node = id => document.getElementById(id);
  const expectedIds = new Set([10,11,12,13,14,20,21,22,23,24,30,31,32,33,34]);
  let lastStatus = null, lastChecked = 0, sampling = false, captureAbort = null;
  let reportUrl = null, historyLoaded = false, wirelessHome = null;

  const statusNodes = new Map();
  const updateText = (element,value) => { if(element && element.textContent!==value)element.textContent=value; };
  const disable = value => { const button=node('simple-refresh');if(button.disabled!==value)button.disabled=value; };
  let paintedAt=-Infinity,paintedKey='',clockSecond=null,clockText='';
  const clockFormat=new Intl.DateTimeFormat('zh-CN',{timeZone:'Asia/Shanghai',hour12:false,hour:'2-digit',minute:'2-digit',second:'2-digit'});
  function statusItem(id, value, detail, state = '') {
    if(!statusNodes.has(id)){const element=node(id);statusNodes.set(id,{element,value:element.querySelector('strong'),detail:element.querySelector('small')});}
    const item=statusNodes.get(id);
    updateText(item.value,value);updateText(item.detail,detail);
    if(item.element.dataset.state!==state)item.element.dataset.state=state;
  }

  function fresh(status) {
    return status && Number.isFinite(status.received) && status.received > 0 &&
      Number.isInteger(status.mcu_seq) && status.mcu_seq >= 0 && status.mcu_seq <= 4294967295 &&
      Number.isFinite(status.feedback_age_ms) && status.feedback_age_ms >= 0 && status.feedback_age_ms < 150;
  }

  function locked(detail, home=homeState()) {
    const status = detail.status;
    const calibration = window.duckWirelessCalibration?.view();
    const wirelessLocked = calibration?.fresh === true && calibration.policyLocked === true &&
      calibration.s?.armed?.active === false && calibration.allOff === true;
    const liveLocked=status?.mode==='wireless_calibration'&&status.policy_locked===true&&status.calibration_supported===true&&
      status.calibration_fresh===true&&Number.isFinite(status.calibration_age_ms)&&status.calibration_age_ms>=0&&status.calibration_age_ms<150&&
      status.all_torque_off_verified===true&&status.torque_may_be_on===false&&status.session_active===false&&status.mcu_state===1;
    return !home.active && fresh(status) && status.enabled === false && (liveLocked || wirelessLocked || detail.readOnly === true && status.mode === 'read_only' &&
      status.commissioning_motion_locked === true);
  }

  function homeState() {
    const view=window.duckWirelessCalibration?.view();
    const state=view?{status:view.s,fresh:view.fresh,homeActive:view.homeActive,homePhase:view.homePhase}:wirelessHome;
    const home=state?.status?.home;
    const liveHome=lastStatus?.status?.home;
    // A historical fault phase is not evidence of a current motion session.
    const stateActive=state?.homeActive===true||home?.active===true;
    const liveActive=liveHome?.active===true;
    return {active:stateActive||liveActive,fresh:stateActive?state?.fresh===true&&home?.supported===true:
      liveActive&&liveHome.supported===true&&fresh(lastStatus?.status)&&Date.now()-lastChecked>=0&&Date.now()-lastChecked<150,
      phase:stateActive?state?.homePhase||home?.phase||'off':liveHome?.phase||'off'};
  }

  function showHomeMotion(home=homeState()) {
    if(!home.active)return false;
    statusItem('simple-motion',!home.fresh||home.phase==='fault'?'停止待确认':({holding:'保持中',moving:'归位中',enabling:'归位中',preparing:'归位准备',prepared:'归位准备',releasing:'卸力中',off:'归位未完成'}[home.phase]||'停止待确认'),'可随时停止');
    disable(true);
    return true;
  }

  function showCalibrationStatus() {
    if(document.hidden||window.DuckWorkflow?.page()!=='servo')return false;
    const view=window.duckWirelessCalibration?.view();
    if(!view?.connected)return false;
    statusItem('simple-link','已连接',view.fresh?'校准反馈':'等待新鲜反馈');
    statusItem('simple-imu','按需读取','切到 IMU 页查看实时姿态');
    if(view.fresh){
      const count=(view.s?.joints||[]).filter(row=>row.valid===true).length;
      statusItem('simple-servos',count+' / 15','校准读回');
      if(!showHomeMotion())statusItem('simple-motion',view.allOff?'已卸力':'待确认',view.allOff?'当前卸力已读回':'等待卸力反馈');
    }else showHomeMotion();
    return true;
  }

  function unavailable(text, detail) {
    lastStatus = null;
    lastChecked = 0;
    paintedKey='unavailable:'+text;
    statusItem('simple-link', text, detail, text === '离线' ? 'fail' : '');
    statusItem('simple-servos', '— / 15', '等待新反馈');
    statusItem('simple-imu', '待确认', '校准待做');
    statusItem('simple-motion', '待确认', '不发送动作');
    showHomeMotion();
    disable(true);
  }

  function showStatus(detail) {
    if(showCalibrationStatus())return;
    if (detail.error || detail.paused) {
      unavailable(detail.error ? '离线' : '待确认', detail.error ? '检查供电和 Wi-Fi' : '读取已暂停');
      return;
    }
    const status = detail.status;
    if (!fresh(status)) {
      unavailable('反馈过期', '无线可达，等待主控');
      return;
    }
    lastStatus = detail;
    lastChecked = detail.checkedAt;
    const count = new Set((Array.isArray(status.servo_ids) ? status.servo_ids : [])
      .filter(id => Number.isInteger(id) && expectedIds.has(id))).size;
    const home=homeState(),isLocked=locked(detail,home),wirelessMode=status.mode==='wireless_calibration'&&status.policy_locked===true;
    const blocked=sampling||!isLocked||status.imu_ok!==true;
    // Ingest every packet above; only presentation is coalesced. Safety transitions
    // bypass the 10Hz display budget, and the click gate reads the latest packet.
    disable(blocked);
    const key=[count,status.imu_ok,isLocked,wirelessMode,home.active,home.fresh,home.phase,
      status.enabled,status.mode,status.policy_locked,status.session_active,status.mcu_state,status.operation,blocked].join('|');
    const now=performance.now();
    if(key===paintedKey&&now-paintedAt<100)return;
    paintedKey=key;paintedAt=now;
    const second=Math.floor(detail.checkedAt/1000);
    if(second!==clockSecond){clockSecond=second;clockText=clockFormat.format(new Date(detail.checkedAt));}
    statusItem('simple-link', '已连接', clockText + ' 更新', 'pass');
    statusItem('simple-servos', count + ' / 15', '已应答', count === 15 ? 'pass' : 'fail');
    statusItem('simple-imu', status.imu_ok === true ? '通信正常' : '未就绪', '校准待做', status.imu_ok === true ? 'pass' : 'fail');
    if(!showHomeMotion(home))statusItem('simple-motion',isLocked?'已锁定':wirelessMode?'校准模式':'未确认锁定',wirelessMode?'小步需授权':'只读调试',isLocked||wirelessMode?'pass':'fail');
    const raw30=Array.isArray(status.raw_pos)?status.raw_pos[10]:null;
    updateText(node('simple-pending'),Number.isInteger(raw30)&&(raw30<0||raw30>4095)
      ?'待处理：ID30 位置编码、IMU 方向、机械零位。':'待处理：IMU 方向、机械零位。');
  }

  function shortTime(value) {
    return new Date(value).toLocaleTimeString('zh-CN', {timeZone: 'Asia/Shanghai', hour12: false, hour: '2-digit', minute: '2-digit'});
  }

  function renderMeasurement(record, historical) {
    const mean = record?.summary?.gyro_deg_s?.mean;
    if (!Array.isArray(mean) || mean.length !== 3 || !mean.every(Number.isFinite)) return;
    node('simple-result').dataset.state = '';
    node('simple-result').textContent = (historical ? '上次 ' : '本次 ') + shortTime(record.recorded_at) +
      '：Y 轴 ' + mean[1].toFixed(2) + '°/s。尚未补偿。';
    if (record.summary.gyro_deg_s.std?.some(value => value > 1) || record.summary.acc_m_s2?.std?.some(value => value > 0.2)) {
      node('simple-result').textContent = (historical ? '上次' : '本次') + '波动较大，请放稳后重测。';
    }
  }

  function makeSummary(rows) {
    const describe = key => {
      const mean = [0,1,2].map(axis => rows.reduce((sum, row) => sum + row[key][axis], 0) / rows.length);
      const std = [0,1,2].map(axis => Math.sqrt(rows.reduce((sum, row) => sum + (row[key][axis] - mean[axis]) ** 2, 0) / rows.length));
      return {mean, std};
    };
    return {
      samples: rows.length,
      gyro_deg_s: describe('gyro'), acc_m_s2: describe('acc'),
      bias_applied: false, installation_axes_verified: false,
      note: '用户点击静置重测；只记录数据，不确认机身是否静止，不写入补偿。',
    };
  }

  function runtimeBusy() {
    return homeState().active || typeof provisionState !== 'undefined' && (provisionState.active === true || provisionPosting === true);
  }

  async function captureStatic() {
    if (sampling || !lastStatus || !locked(lastStatus) || Date.now() - lastChecked > 8000 || runtimeBusy()) return;
    const target = window.DuckLiveRuntime?.url() || '';
    if (typeof safeProvisionUrl !== 'function' || !safeProvisionUrl(target)) return;
    sampling = true;
    window.DUCK_STATIC_SAMPLING = true;
    window.dispatchEvent?.(new CustomEvent('duck:static-sampling'));
    if (typeof cancelProvisionLive === 'function') cancelProvisionLive();
    captureAbort = new AbortController();
    const abort = captureAbort;
    const button = node('simple-refresh'), result = node('simple-result');
    button.disabled = true;
    result.dataset.state = '';
    result.textContent = '保持不动，正在测量…';
    const rows = [], started = performance.now();
    let next = started, instance = null, previousSequence = null;
    try {
      while (performance.now() - started < 10000) {
        if (document.hidden || runtimeBusy()) throw Error('测量已暂停，请重新静置重测。');
        const timer = setTimeout(() => abort.abort(), 3500);
        let response;
        try {
          response = await fetch('/api/board-status?url=' + encodeURIComponent(target), {cache: 'no-store', signal: abort.signal});
        } finally { clearTimeout(timer); }
        if (!response.ok) throw Error('连接中断，请重新测量。');
        const data = await response.json(), status = data.status;
        if (document.hidden || runtimeBusy() || abort.signal.aborted) throw Error('测量已暂停，请重新静置重测。');
        const detail = {status, readOnly: data.read_only, checkedAt: Date.now()};
        showStatus(detail);
        if (typeof CustomEvent === 'function' && typeof window.dispatchEvent === 'function') window.dispatchEvent(new CustomEvent('duck:live-status', {detail}));
        if (!locked(detail) || status.imu_ok !== true) throw Error('新反馈或运动锁未确认，测量停止。');
        if (![status.gyro, status.acc].every(v => Array.isArray(v) && v.length === 3 && v.every(Number.isFinite))) throw Error('IMU 数据不完整，测量停止。');
        if (rows.length && (status.app_instance_id !== instance || status.mcu_seq <= previousSequence)) throw Error('程序重启或反馈未前进，请重新测量。');
        instance = status.app_instance_id;
        previousSequence = status.mcu_seq;
        rows.push({elapsed_s: (performance.now() - started) / 1000, sequence: status.mcu_seq, gyro: status.gyro, acc: status.acc});
        button.textContent = '测量中 · ' + Math.max(1, Math.ceil((10000 - (performance.now() - started)) / 1000)) + ' 秒';
        next += 200;
        await new Promise(resolve => setTimeout(resolve, Math.max(0, next - performance.now())));
      }
      if (rows.length < 10) throw Error('有效数据不足，请重新测量。');
      if (document.hidden || runtimeBusy() || abort.signal.aborted) throw Error('测量已暂停，请重新静置重测。');
      const record = {recorded_at: new Date().toISOString(), read_only: true, summary: makeSummary(rows), samples: rows};
      const serialized = JSON.stringify(record);
      try { localStorage.setItem('duck-imu-static', serialized); } catch { /* Download remains available. */ }
      historyLoaded = true;
      renderMeasurement(record, false);
      if (record.summary.gyro_deg_s.std.some(value => value > 1) || record.summary.acc_m_s2.std.some(value => value > 0.2)) {
        result.textContent = '本次波动较大，请放稳后重测。';
      }
      if (reportUrl) URL.revokeObjectURL(reportUrl);
      reportUrl = URL.createObjectURL(new Blob([JSON.stringify(record, null, 2)], {type: 'application/json'}));
      node('simple-report').href = reportUrl;
      node('simple-report').hidden = false;
    } catch (error) {
      unavailable('待确认', '测量未完成');
      result.dataset.state = 'error';
      result.textContent = error.name === 'AbortError' ? '测量中断，请保持静止后重测。' : error.message;
    } finally {
      sampling = false;
      window.DUCK_STATIC_SAMPLING = false;
      window.dispatchEvent?.(new CustomEvent('duck:static-sampling'));
      if (captureAbort === abort) captureAbort = null;
      button.textContent = '静置重测 · 10 秒';
      button.disabled = !lastStatus || !locked(lastStatus) || Date.now() - lastChecked > 8000 || runtimeBusy();
      if (typeof readProvisionLive === 'function') readProvisionLive();
    }
  }

  window.addEventListener('duck:live-status', event => {
    if (!sampling) showStatus(event.detail);
  });
  window.addEventListener('duck:wireless-calibration',event=>{
    wirelessHome=event.detail;
    if(showCalibrationStatus())return;
    if(homeState().active){if(captureAbort)captureAbort.abort();showHomeMotion();}
    else if(lastStatus&&!sampling)showStatus(lastStatus);
  });
  window.addEventListener('duck:provision-state', event => {
    if (event.detail.active === true) {
      if (captureAbort) captureAbort.abort();
      unavailable('待确认', '配置期间暂停读取');
    }
  });
  document.addEventListener('visibilitychange', () => {
    if (document.hidden) {
      if (captureAbort) captureAbort.abort();
      unavailable('待确认', '页面隐藏');
    }
  });
  setInterval(() => {
    if(showCalibrationStatus())return;
    if (lastChecked && Date.now() - lastChecked > 8000) unavailable('反馈过期', '等待新反馈');
  }, 1000);
  node('simple-refresh').addEventListener('click', captureStatic);
  (async () => {
    if(node('simple-refresh').hidden)return;
    try {
      const saved = localStorage.getItem('duck-imu-static');
      if (saved) { renderMeasurement(JSON.parse(saved), true); historyLoaded = true; }
      const response = await fetch('/assets/imu-static-last.json', {cache: 'no-store'});
      if (response.ok && !historyLoaded && !sampling) renderMeasurement(await response.json(), true);
    } catch {
      if (!historyLoaded) node('simple-result').textContent = '点击按钮开始测量。';
    }
  })();
})();
