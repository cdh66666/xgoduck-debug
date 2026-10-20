/* Draw only observed IMU samples; never synthesize gaps or write calibration. */
(() => {
  'use strict';
  const M = window.XgoImuMath, get = id => document.getElementById(id);
  const colors = ['#256fbb','#bc6b22','#835bb3'];
  let rows = [], previousSequence = null, previousInstance = null;
  let lastArrival = 0, lastNewSample = 0, visibleData = false, gap = false;
  let modelStatus = null, modelObservedAt = 0, currentPage = 'imu';
  let modelLastMode=null,connectionLastMode=null;
  let dataRate=null,controlRate=null,renderRate=null,renderRateAt=0;
  let lastChartAt=-Infinity,lastStatsAt=-Infinity,lastValuesAt=-Infinity;
  let presentation=null,paintQueued=false,forcePaint=false,graphicsState='ready';
  function showRates(){text('imu-live-rate','数据 '+(Number.isFinite(dataRate)?dataRate.toFixed(1):'—')+' Hz · 画面 '+(graphicsState==='lost'?'失效':(Number.isFinite(renderRate)?renderRate.toFixed(0):'—')+' FPS')+' · 主控 '+(Number.isFinite(controlRate)?controlRate.toFixed(0):'—')+' Hz');}
  function scheduleGraphicsRecovery(){
    // Three restores this same context in place. Navigating the iframe here
    // destroys its last pose and canvas and adds a second visible blackout.
    const frame=get('imu-duck-frame');if(frame)frame.dataset.graphics=graphicsState;
  }
  let connectionStatus=null,connectionObservedAt=0,connectionSequence=null,connectionInstance=null,connectionLastNew=null;
  const connectionIds=[10,11,12,13,14,20,21,22,23,24,30,31,32,33,34];
  function connectionMessage(mode,status,reason,observedAt=connectionObservedAt) {
    const frame=get('connection-duck-frame');
    if(!frame?.contentWindow||typeof frame.contentWindow.postMessage!=='function')return;
    const paused=document.hidden||currentPage!=='link';
    if(paused&&connectionLastMode==='paused')return;
    connectionLastMode=paused?'paused':mode;
    const s=status?{mcu_seq:status.mcu_seq,received:status.received,feedback_age_ms:status.feedback_age_ms,
      app_instance_id:status.app_instance_id,servo_ids:status.servo_ids?.slice(),imu_ok:status.imu_ok}:null;
    frame.contentWindow.postMessage({type:'connection-model-state',version:1,mode:paused?'paused':mode,
      status:paused?null:s,reason:paused?'页面已暂停':reason,observedAtMs:observedAt},location.origin);
  }
  function clearConnection(reason) {connectionStatus=null;connectionMessage(reason==='读取暂停'?'paused':'stale',null,reason);}
  function consumeConnection(detail) {
    const s=detail?.status;
    if(detail?.error||detail?.paused||!validFresh(s)||typeof s.app_instance_id!=='string'||!s.app_instance_id||typeof s.imu_ok!=='boolean'||
      !Array.isArray(s.servo_ids)||s.servo_ids.length>15||new Set(s.servo_ids).size!==s.servo_ids.length||s.servo_ids.some(id=>!Number.isInteger(id)||!connectionIds.includes(id))){
      clearConnection(detail?.paused?'读取暂停':detail?.error?'离线':'反馈无效');return;
    }
    const now=performance.now(),step=M.sequenceStep(connectionSequence,s.mcu_seq,s.app_instance_id===connectionInstance);
    if(step!=='duplicate'){
      connectionStatus={mcu_seq:s.mcu_seq,received:s.received,feedback_age_ms:s.feedback_age_ms,app_instance_id:s.app_instance_id,servo_ids:s.servo_ids.slice(),imu_ok:s.imu_ok};
      connectionObservedAt=Date.now();connectionLastNew=now-s.feedback_age_ms;connectionSequence=s.mcu_seq;connectionInstance=s.app_instance_id;
    }
    if(connectionLastNew===null||now-connectionLastNew>1500){clearConnection('反馈未前进');return;}
    connectionMessage(connectionStatus?'live':'stale',connectionStatus,connectionStatus?'实时通信':'等待反馈');
  }
  function modelMessage(mode,status,reason,observedAt=modelObservedAt) {
    const frame=get('imu-duck-frame');
    if(!frame?.contentWindow||typeof frame.contentWindow.postMessage!=='function')return;
    const paused=document.hidden||currentPage!=='imu';
    if(paused&&modelLastMode==='paused')return;
    modelLastMode=paused?'paused':mode;
    const s=status?{acc:status.acc?.slice(),gyro:status.gyro?.slice(),mcu_seq:status.mcu_seq,
      feedback_age_ms:status.feedback_age_ms,received:status.received,imu_ok:status.imu_ok,app_instance_id:status.app_instance_id,mcu_imu_us:status.mcu_imu_us,mcu_restarts:status.mcu_restarts,
      display_observed_monotonic_ms:status.display_observed_monotonic_ms,imu_attitude:status.imu_attitude}:null;
    frame.contentWindow.postMessage({type:'imu-model-state',version:1,mode:paused?'paused':mode,
      status:paused?null:s,reason:paused?'页面已暂停':reason,observedAtMs:observedAt},location.origin);
  }
  const plots = [
    {id:'imu-gyro-chart',key:'gyro',unit:'°/s',hover:'imu-gyro-hover',refs:[0]},
    {id:'imu-acc-chart',key:'acc',unit:'m/s²',hover:'imu-acc-hover',refs:[-9.80665,0,9.80665]},
  ];
  const hasDiagnostics=plots.some(plot=>get(plot.id));
  const validFresh = s => s && Number.isInteger(s.mcu_seq) && s.mcu_seq>=0 && s.mcu_seq<=4294967295 &&
    Number.isFinite(s.received) && s.received>0 && Number.isFinite(s.feedback_age_ms) && s.feedback_age_ms>=0 && s.feedback_age_ms<150;
  const text = (id,value) => { const element=get(id);if(element&&element.textContent!==value)element.textContent=value; };
  const format = value => Number.isFinite(value) ? value.toFixed(2) : '—';

  function clearCurrent(message) {
    dataRate=null;controlRate=null;renderRate=null;
    if (visibleData && !gap) rows.push({t:performance.now(),gap:true});
    gap = true;
    visibleData = false;
    for (const axis of ['x','y','z']) {text('imu-gyro-'+axis,'—');text('imu-acc-'+axis,'—');}
    text('imu-roll','—');text('imu-pitch','—');text('imu-signal','等待新鲜数据');
    text('imu-live-rate',message);text('imu-pose-state',message);
    text('imu-statistics','等待新鲜数据');
    text('imu-gyro-hover','读取暂停；曲线仅保留历史');
    text('imu-acc-hover','读取暂停；曲线仅保留历史');
    if(get('imu-pose-view'))get('imu-pose-view').dataset.live='false';
    modelStatus=null;presentation=null;
    modelMessage(message==='读取暂停'?'paused':'stale',null,message);
    schedulePaint();
  }

  function consume(detail) {
    const observed=detail?.displayObservation||detail,s=observed?.status;
    const validObservation=s&&Number.isInteger(s.mcu_seq)&&s.mcu_seq>=0&&s.mcu_seq<=0xffffffff&&Number.isFinite(s.received)&&s.received>0&&Number.isFinite(s.feedback_age_ms)&&s.feedback_age_ms>=0;
    if (detail?.paused || detail?.error&&!detail?.displayObservation || !validObservation || s.imu_ok!==true || !M.vector(s.acc) || !M.vector(s.gyro)) {
      clearCurrent(detail?.paused?'读取暂停':detail?.error?'离线':'IMU 等待数据');
      return;
    }
    const now = performance.now();
    lastArrival = now;
    const step = M.sequenceStep(previousSequence,s.mcu_seq,s.app_instance_id===previousInstance);
    if(step!=='duplicate'){modelStatus=s;modelObservedAt=Date.now();}
    modelMessage('live',s,'重力姿态',modelObservedAt);
    if (step==='duplicate') {
      if (now-lastNewSample>1500) clearCurrent('反馈未前进');
      return;
    }
    if (step==='reset') rows=[];
    previousSequence=s.mcu_seq;
    previousInstance=s.app_instance_id;
    lastNewSample=now;
    // The simple pose page has no plots or raw-value widgets. Forward every
    // actual observation to the model without retaining unused chart history.
    if(!hasDiagnostics){visibleData=true;gap=false;return;}
    if (gap && rows.length && !rows[rows.length-1].gap) rows.push({t:now,gap:true});
    gap=false;
    const clockUs=Number.isInteger(s.mcu_imu_us)&&s.mcu_imu_us>=0&&s.mcu_imu_us<=0xffffffff?s.mcu_imu_us:
      Number.isInteger(s.mcu_sample_us)&&s.mcu_sample_us>=0&&s.mcu_sample_us<=0xffffffff?s.mcu_sample_us:null;
    rows.push({t:now,acc:s.acc.slice(),gyro:s.gyro.slice(),sequence:s.mcu_seq,sampleUs:clockUs});
    rows=rows.filter(row=>row.t>=now-21000);
    visibleData=true;
    presentation=s;schedulePaint();
  }

  function paintValues(s,now){
    text('imu-gyro-hover','原始值，未补偿');text('imu-acc-hover','原始值，未补偿');
    for (const [i,axis] of ['x','y','z'].entries()) {
      text('imu-gyro-'+axis,format(s.gyro[i]));text('imu-acc-'+axis,format(s.acc[i]));
    }
    const a=M.attitude(s.acc);
    if(get('imu-pose-view'))get('imu-pose-view').dataset.live=String(!!a);
    if (a) {
      text('imu-pitch',format(a.pitch)+'°');text('imu-roll',format(a.roll)+'°');
      get('imu-pitch-body')?.setAttribute('transform','rotate('+a.pitch+' 90 100)');
      get('imu-roll-body')?.setAttribute('transform','rotate('+a.roll+' 270 100)');
      text('imu-pose-state',a.inverted?'朝向超过 90°，核对安装轴':'总倾角 '+format(a.tilt)+'°');
      text('imu-signal','重力模长 '+format(a.norm)+' m/s² · 反馈 '+format(s.feedback_age_ms)+' ms');
    } else {text('imu-pitch','—');text('imu-roll','—');text('imu-pose-state','加速度无效，倾角未知');text('imu-signal','加速度无效，重力模长未知');}
    const recent=rows.filter(row=>!row.gap&&row.t>=now-5000);
    if(now-lastStatsAt>=200){
      const stats=M.describe(recent,'gyro');
      if(stats) text('imu-statistics',['X','Y','Z'].map((axis,i)=>axis+' '+format(stats.mean[i])+' / '+format(stats.std[i])).join('　')+' °/s');
      lastStatsAt=now;
    }
    const clocks=recent.filter(row=>row.sampleUs!==null);
    const sourceSpan=clocks.length>1?((clocks.at(-1).sampleUs-clocks[0].sampleUs)>>>0)/1000:0;
    const sourceCount=new Set(clocks.map(row=>row.sampleUs)).size;
    const rate=clocks.length===recent.length?(sourceSpan>0&&sourceSpan<=10000?(sourceCount-1)*1000/sourceSpan:null):
      recent.length>1?(recent.length-1)*1000/(recent.at(-1).t-recent[0].t):null;
    dataRate=rate;controlRate=s.feedback_hz;showRates();
  }
  function schedulePaint(force=false){
    forcePaint=forcePaint||force;
    if(paintQueued||document.hidden||currentPage!=='imu')return;
    paintQueued=true;
    requestAnimationFrame(()=>{
      paintQueued=false;const force=forcePaint;forcePaint=false;
      if(document.hidden||currentPage!=='imu')return;
      const now=performance.now();
      if(visibleData&&presentation&&(force||now-lastValuesAt>=100)){paintValues(presentation,now);lastValuesAt=now;}
      drawAll(force);
    });
  }

  let chartPalette;
  function updateChartPalette(){
    const style=typeof getComputedStyle==='function'?getComputedStyle(document.body):null;
    const rgb=style?.backgroundColor?.match(/\d+(?:\.\d+)?/g)?.slice(0,3).map(Number);
    const dark=document.body?.dataset?.theme==='dark'||style?.colorScheme==='dark'||(rgb?.length===3&&rgb.reduce((a,b)=>a+b,0)<220);
    const palette=dark?['#43d2e8','#f2b163','#a293ef']:colors;
    const ink=dark?'#7e9aab':'#627787',grid=dark?'#203746':'#dbe4e9',reference=dark?'#416377':'#a9bbc6';
    chartPalette={palette,ink,grid,reference};
  }
  updateChartPalette();
  if(typeof MutationObserver==='function')new MutationObserver(()=>{updateChartPalette();schedulePaint(true);}).observe(document.body,{attributes:true,attributeFilter:['class','style','data-theme']});

  function draw(plot) {
    const canvas=get(plot.id);
    if(!canvas)return;
    if(plot.key==='acc'&&get('imu-details')?.open!==true)return;
    const bounds=plot.bounds;
    if(!bounds||bounds.width<10||bounds.height<10)return;
    const w=bounds.width,h=bounds.height,ratio=window.devicePixelRatio||1;
    if(canvas.width!==Math.round(w*ratio)||canvas.height!==Math.round(h*ratio)){canvas.width=Math.round(w*ratio);canvas.height=Math.round(h*ratio);}
    const ctx=canvas.getContext('2d');ctx.setTransform(ratio,0,0,ratio,0,0);ctx.clearRect(0,0,w,h);
    const {palette,ink,grid,reference}=chartPalette;
    const pad={left:48,right:13,top:13,bottom:27},pw=w-pad.left-pad.right,ph=h-pad.top-pad.bottom;
    const now=performance.now(),start=now-20000,points=rows.filter(row=>row.t>=start),valid=points.filter(row=>!row.gap);
    const values=valid.flatMap(row=>row[plot.key]).concat(plot.refs);
    let lo=Math.min(...values),hi=Math.max(...values);
    if(!Number.isFinite(lo)||!Number.isFinite(hi)){lo=-1;hi=1;}
    const minimum=plot.key==='gyro'?1:2;
    if(hi-lo<minimum){const center=(hi+lo)/2;lo=center-minimum/2;hi=center+minimum/2;}
    const range=hi-lo;lo-=range*0.08;hi+=range*0.08;
    const x=t=>pad.left+(t-start)/20000*pw,y=v=>pad.top+(hi-v)/(hi-lo)*ph;
    ctx.font='11px "Microsoft YaHei",sans-serif';ctx.fillStyle=ink;ctx.strokeStyle=grid;ctx.lineWidth=1;
    for(let i=0;i<4;i++){const v=lo+(hi-lo)*i/3,py=y(v);ctx.beginPath();ctx.moveTo(pad.left,py);ctx.lineTo(w-pad.right,py);ctx.stroke();ctx.textAlign='right';ctx.textBaseline='middle';ctx.fillText(v.toFixed(1),pad.left-6,py);}
    for(let i=0;i<3;i++){const px=pad.left+pw*i/2;ctx.textAlign=i===0?'left':i===2?'right':'center';ctx.textBaseline='top';ctx.fillText(i===2?'现在':(20-i*10)+' 秒前',px,h-pad.bottom+8);}
    ctx.save();ctx.beginPath();ctx.rect(pad.left,pad.top,pw,ph);ctx.clip();
    ctx.strokeStyle=reference;ctx.setLineDash([4,4]);
    for(const ref of plot.refs){if(ref<lo||ref>hi)continue;ctx.beginPath();ctx.moveTo(pad.left,y(ref));ctx.lineTo(w-pad.right,y(ref));ctx.stroke();}
    ctx.setLineDash([]);
    for(let axis=0;axis<3;axis++){
      ctx.strokeStyle=palette[axis];ctx.lineWidth=1.7;ctx.beginPath();let drawing=false,lastT=null;
      for(const row of points){if(row.gap){drawing=false;lastT=null;continue;}if(lastT!==null&&row.t-lastT>1200)drawing=false;const px=x(row.t),py=y(row[plot.key][axis]);if(drawing)ctx.lineTo(px,py);else ctx.moveTo(px,py);drawing=true;lastT=row.t;}
      ctx.stroke();
      const latest=valid[valid.length-1];if(latest){ctx.beginPath();ctx.arc(x(latest.t),y(latest[plot.key][axis]),2.7,0,Math.PI*2);ctx.fillStyle=palette[axis];ctx.fill();}
    }
    ctx.restore();
    if(!valid.length){ctx.fillStyle=ink;ctx.textAlign='center';ctx.fillText('等待真实数据',pad.left+pw/2,pad.top+ph/2);}
    plot.geometry={pad,w,pw,start};
  }

  function drawAll(force=false){
    if(document.hidden||currentPage!=='imu')return;
    const now=performance.now();
    if(!force&&now-lastChartAt<1000/30)return;
    lastChartAt=now;for(const plot of plots)draw(plot);
  }
  for(const plot of plots){
    const canvas=get(plot.id);
    if(!canvas)continue;
    canvas.addEventListener('pointermove',event=>{
      if(!plot.geometry)return;
      const rect=canvas.getBoundingClientRect(),g=plot.geometry;
      const fraction=Math.max(0,Math.min(1,(event.clientX-rect.left-g.pad.left)/g.pw)),time=g.start+fraction*20000;
      const candidates=rows.filter(row=>!row.gap);
      if(!candidates.length)return;
      const row=candidates.reduce((a,b)=>Math.abs(a.t-time)<Math.abs(b.t-time)?a:b);
      text(plot.hover,['X','Y','Z'].map((axis,i)=>axis+' '+format(row[plot.key][i])).join('　')+' '+plot.unit+' · '+Math.max(0,(performance.now()-row.t)/1000).toFixed(1)+' 秒前');
    });
    const rememberBounds=rect=>{plot.bounds={width:rect.width,height:rect.height};schedulePaint(true);};
    if(typeof ResizeObserver==='function')new ResizeObserver(entries=>{const entry=entries.find(row=>row.target===canvas)||entries[0];if(entry?.contentRect)rememberBounds(entry.contentRect);}).observe(canvas);
    else window.addEventListener('resize',()=>rememberBounds(canvas.getBoundingClientRect()));
    rememberBounds(canvas.getBoundingClientRect());
  }
  window.addEventListener('duck:live-status',event=>{
    if(currentPage==='link')consumeConnection(event.detail);
    if(currentPage==='imu'&&event.detail?.summaryOnly!==true&&window.DuckIMUDatagramEnabled!==true)consume(event.detail);
  });
  window.addEventListener('duck:imu-status',event=>{
    if(currentPage==='imu'&&window.DuckIMUDatagramEnabled===true)consume(event.detail);
  });
  window.addEventListener('duck:imu-mode',event=>{
    if(event.detail?.enabled===true)clearCurrent('等待新鲜数据');
  });
  window.addEventListener('duck:page',event=>{
    const page=event.detail?.page||currentPage;
    if(page!==currentPage){
      modelStatus=null;presentation=null;visibleData=false;gap=true;previousSequence=null;previousInstance=null;
      modelLastMode=null;
    }
    currentPage=page;
    modelMessage(modelStatus?'live':currentPage==='imu'?'stale':'paused',modelStatus,'等待新鲜数据');
    connectionMessage(connectionStatus?'live':'stale',connectionStatus,'等待反馈');schedulePaint(true);scheduleGraphicsRecovery();
  });
  window.addEventListener('message',event=>{
    const connectionFrame=get('connection-duck-frame');
    if(connectionFrame?.contentWindow&&event.origin===location.origin&&event.source===connectionFrame.contentWindow&&event.data?.type==='connection-model-ready'&&event.data.version===1&&event.data.meshes===15){
      connectionFrame.dataset.loaded='true';connectionLastMode=null;connectionMessage(connectionStatus?'live':'stale',connectionStatus,'等待反馈');return;
    }
    const frame=get('imu-duck-frame');
    if(frame?.contentWindow&&event.origin===location.origin&&event.source===frame.contentWindow&&event.data?.type==='imu-model-graphics-state'&&event.data.version===1&&['ready','waiting','lost'].includes(event.data.state)){
      graphicsState=event.data.state;renderRate=null;frame.dataset.loaded=String(event.data.state==='ready'&&event.data.loaded===true);
      showRates();scheduleGraphicsRecovery();return;
    }
    if(frame?.contentWindow&&event.origin===location.origin&&event.source===frame.contentWindow&&event.data?.type==='imu-model-render-metrics'&&event.data.version===1&&event.data.measured===true&&Number.isFinite(event.data.fps)&&event.data.fps>0&&event.data.fps<=240){
      if(currentPage==='imu'&&!document.hidden&&visibleData&&graphicsState==='ready'){renderRate=event.data.fps;renderRateAt=performance.now();schedulePaint();}return;
    }
    if(!frame?.contentWindow||event.origin!==location.origin||event.source!==frame.contentWindow||event.data?.type!=='imu-model-ready'||event.data.version!==1||event.data.meshes!==15)return;
    frame.dataset.loaded=String(graphicsState==='ready');
    modelLastMode=null;
    modelMessage(modelStatus?'live':'paused',modelStatus,'等待反馈');
  });
  document.addEventListener('visibilitychange',()=>{if(document.hidden){clearCurrent('读取暂停');clearConnection('读取暂停');}scheduleGraphicsRecovery();});
  setInterval(()=>{if(renderRate!==null&&performance.now()-renderRateAt>2500){renderRate=null;if(visibleData)showRates();}if(connectionStatus&&connectionLastNew!==null&&performance.now()-connectionLastNew>1500)clearConnection('反馈过期');if(visibleData&&performance.now()-lastArrival>1800)clearCurrent('反馈过期');if(!visibleData&&rows.length){rows=rows.filter(row=>row.t>=performance.now()-21000);schedulePaint();}},500);
  schedulePaint();
})();
