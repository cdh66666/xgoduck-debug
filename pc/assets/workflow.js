/* Navigation and real readbacks only. Existing physical action gates remain intact. */
(() => {
  'use strict';
  const get=id=>document.getElementById(id),pages=['numbering','install','imu','models'];
  const defaultPage='imu',pageKey='duck-workflow-page-v4';
  const joints=[
    [10,'左髋转向'],[11,'左髋侧摆'],[12,'左髋前后摆'],[13,'左膝'],[14,'左踝'],
    [20,'右髋转向'],[21,'右髋侧摆'],[22,'右髋前后摆'],[23,'右膝'],[24,'右踝'],
    [30,'颈俯仰'],[31,'头俯仰'],[32,'头转向'],[33,'头侧倾'],[34,'嘴巴'],
  ];
  let selected=10,lastStatus=null,wirelessCalibration=null,currentPage=defaultPage,lastCheckedAt=0,wirelessCheckedAt=0;
  const setText=(element,value)=>{if(element&&element.textContent!==value)element.textContent=value;};
  const text=(id,value)=>setText(get(id),value);
  const attribute=(element,key,value)=>{if(element&&element.getAttribute(key)!==String(value))element.setAttribute(key,String(value));};
  const dataset=(element,key,value)=>{if(element&&element.dataset[key]!==String(value))element.dataset[key]=String(value);};
  let selectedPaintedAt=-Infinity,selectedPaintedKey='',batteryPaintedAt=-Infinity,batteryPaintedKey='';
  let batteryReading=null;
  const SERVO_VOLTAGE_CONFIRM_MS=300;
  let servoVoltage={shown:null,candidate:null,candidateSince:0,lastSampleAt:0,lastFrameAt:0,lastSeq:null,source:''};
  let charging=true;
  try{charging=localStorage.getItem('duck-battery-returned-20261012')!=='yes';}catch{}
  const fresh=s=>s&&Number.isFinite(s.feedback_age_ms)&&s.feedback_age_ms>=0&&s.feedback_age_ms<150&&Number.isInteger(s.mcu_seq)&&Number.isFinite(s.received)&&s.received>0;

  function showPage(name){
    const calibration=name==='servo',connection=name==='link';
    if(calibration||connection)name='imu';
    if(!pages.includes(name))name=defaultPage;
    currentPage=name;
    if(document.body)document.body.dataset.page=name;
    syncModelsFrame();
    syncNumberingFrame();
    for(const page of pages){const panel=get('workflow-'+page);if(panel)panel.hidden=page!==name;}
    for(const button of get('workflow-tabs').querySelectorAll('button')){
      const active=button.dataset.page===name;button.classList.toggle('active',active);
      if(active)button.setAttribute('aria-current','page');else button.removeAttribute('aria-current');
    }
    try{localStorage.setItem(pageKey,name);}catch{}
    if(name==='imu'){renderSelected(true);renderCalibrationProgress();}
    if(connection&&get('debug-connection'))get('debug-connection').open=true;
    window.dispatchEvent(new CustomEvent('duck:page',{detail:{page:name}}));
    if(calibration)window.dispatchEvent(new CustomEvent('duck:calibration-panel',{detail:{open:true}}));
    window.scrollTo({top:0,behavior:'instant'});
  }

  function syncNumberingFrame(){
    const frame=get('numbering-frame');
    if(!frame)return;
    if(currentPage==='numbering'&&!frame.getAttribute('src')&&frame.dataset.src)frame.setAttribute('src',frame.dataset.src);
    const origin=window.location?.origin;
    if(frame.contentWindow&&origin)frame.contentWindow.postMessage({type:'duck-numbering-visibility',version:1,visible:currentPage==='numbering'&&!document.hidden},origin);
  }

  function syncModelsFrame(){
    const frame=get('models-frame'),origin=window.location?.origin;
    if(!frame)return;
    if(currentPage==='models'&&!frame.getAttribute('src')&&frame.dataset.src)frame.setAttribute('src',frame.dataset.src);
    if(frame.contentWindow&&origin)frame.contentWindow.postMessage({type:'duck-model-library-visibility',version:1,visible:currentPage==='models'&&!document.hidden},origin);
  }

  function renderSelected(force=false){
    if(!force&&(document.hidden||currentPage!=='imu'))return;
    const index=joints.findIndex(j=>j[0]===selected),name=joints[index][1];

    const joint=wirelessCalibration?.status?.joints?.find(row=>row.id===selected);
    const helpers=window.DuckWirelessCalibration;
    const raw=wirelessCalibration?(helpers?.observedPosition?helpers.observedPosition(joint):joint?.signed_steps??joint?.position):lastStatus?.raw_pos?.[index];
    const calibrationFresh=!document.hidden&&wirelessCalibration?.fresh===true&&wirelessCheckedAt>0&&Date.now()-wirelessCheckedAt>=0&&Date.now()-wirelessCheckedAt<1800;
    const responded=wirelessCalibration?calibrationFresh&&joint?.fresh===true:lastStatus&&fresh(lastStatus)&&Array.isArray(lastStatus.servo_ids)&&lastStatus.servo_ids.includes(selected);

    const records=wirelessCalibration?.records||{};
    const record=Array.isArray(records)?records.find(row=>row.id===selected):records[selected];
    const zero=typeof record==='number'?record:record?.position??record?.zero_pos??record?.raw_position;

    const pose=[12,22].includes(selected)?'托稳灰机身，绕'+selected+'转橙大腿；侧看膝'+(selected+1)+'在髋'+selected+'后下方，对齐蓝虚线；嘴朝向为前。':selected===34?'按已确认的照片轻闭嘴，别压紧。':'灰色固定，橙色按图对齐。';
    const diagnosis=responded&&wirelessCalibration&&joint?.valid!==true?(helpers?.jointReadout?helpers.jointReadout(joint,true):'ID '+selected+'在线，编码 '+raw+(Number.isInteger(raw)&&(raw<0||raw>4095)?'，超出旧单圈校验':'，编码域待核对')):'';
    const home=wirelessCalibration?.status?.home;
    const homeUnreleased=wirelessCalibration?.homeActive===true||home?.active===true||home?.supported===true&&home.release_verified!==true&&['preparing','prepared','enabling','moving','holding','releasing','fault'].includes(home.phase);
    const paintKey=[selected,responded,joint?.valid,homeUnreleased,home?.phase,zero].join('|');
    const now=performance.now();
    if(!force&&paintKey===selectedPaintedKey&&now-selectedPaintedAt<100)return;
    selectedPaintedAt=now;selectedPaintedKey=paintKey;
    text('servo-selected-name','ID '+selected+' · '+name);
    text('servo-selected-position',responded&&Number.isInteger(raw)?String(raw):'—');
    text('servo-selected-zero',Number.isInteger(zero)?String(zero):'待记录');
    text('servo-guidance',homeUnreleased?'归位中或停止待确认，请保持托稳；可随时停止。':(diagnosis?diagnosis+'；':'')+pose);
    for(const button of get('servo-picker-groups').querySelectorAll('button'))attribute(button,'aria-pressed',String(Number(button.dataset.id)===selected));
  }

  function renderCalibrationProgress(){
    if(document.hidden||currentPage!=='imu')return;
    const records=wirelessCalibration?.records||{};
    const count=joints.filter(([id])=>window.DuckWirelessCalibration?.recordValid?window.DuckWirelessCalibration.recordValid(records[id]):Number.isInteger(records[id]?.position)&&records[id].position>=0&&records[id].position<=4095).length;
    text('servo-progress',count||wirelessCalibration?.status?.records?'已记录 '+count+' / 15 · 掉电/方向/模型应用待核验':'零位记录待读取');
  }

  function selectJoint(id){
    if(!joints.some(j=>j[0]===id))return;
    selected=id;
    window.dispatchEvent(new CustomEvent('duck:joint',{detail:{id}}));
    renderSelected(true);
  }
  window.DuckWorkflow=Object.freeze({selectedJoint:()=>selected,selectJoint:id=>selectJoint(Number(id)),page:()=>currentPage,openPage:showPage});

  function displayServoVoltage(s,p){
    const now=Date.now(),sampleAt=now-p.age_ms,units=Math.round(p.voltage_v*10);
    const source=String(s.app_instance_id??'')+'/'+String(s.mcu_restarts??'');
    if(servoVoltage.shown===null||servoVoltage.source!==source){
      servoVoltage={shown:units,candidate:null,candidateSince:0,lastSampleAt:sampleAt,lastFrameAt:now,lastSeq:s.mcu_seq,source};
      return units/10;
    }
    const seqDelta=(s.mcu_seq-servoVoltage.lastSeq)>>>0;
    if(seqDelta>0&&seqDelta<0x80000000&&sampleAt>servoVoltage.lastSampleAt){
      if(now-servoVoltage.lastFrameAt>=1500){servoVoltage.shown=units;servoVoltage.candidate=null;}
      else{
        const direction=Math.sign(units-servoVoltage.shown);
        if(direction===0)servoVoltage.candidate=null;
        else if(direction!==servoVoltage.candidate){servoVoltage.candidate=direction;servoVoltage.candidateSince=sampleAt;}
        else if(sampleAt-servoVoltage.candidateSince>=SERVO_VOLTAGE_CONFIRM_MS){
          // Follow a sustained rise or fall even when each observed sample changes by 0.1 V.
          servoVoltage.shown=units;servoVoltage.candidate=null;
        }
      }
      servoVoltage.lastSampleAt=sampleAt;
      servoVoltage.lastSeq=s.mcu_seq;
      servoVoltage.lastFrameAt=now;
    }
    return servoVoltage.shown/10;
  }

  function showBattery(s,force=false){
    const b=s?.battery,p=s?.power_observation_validated===true?s.servo_power:null;
    const sourceFresh=!document.hidden&&(s?.power_observation_validated===true||fresh(s));
    const packFresh=sourceFresh&&b?.supported===true&&['bms','battery_adc'].includes(b.source)&&Number.isFinite(b.age_ms)&&b.age_ms>=0&&b.age_ms<1500;
    const packValid=packFresh&&Number.isFinite(b.voltage_v)&&b.voltage_v>0;
    const busValid=sourceFresh&&p?.supported===true&&p.source==='servo_supply'&&Number.isFinite(p.age_ms)&&p.age_ms>=0&&p.age_ms<1500&&Number.isFinite(p.voltage_v)&&p.voltage_v>0;
    const reported=packFresh&&b.source==='bms'&&Number.isFinite(b.percent)&&b.percent>=0&&b.percent<=100?b.percent:null;
    if(!busValid||packValid)servoVoltage.candidate=null;
    if(packValid)servoVoltage.source='';
    const servoDisplay=busValid&&!packValid?p.voltage_v:null;
    const servoObserved=servoDisplay!==null;
    const voltage=packValid?b.voltage_v:servoObserved?servoDisplay:null;
    if(!document.hidden&&reported===null&&voltage===null&&batteryReading){
      batteryPaintedKey='cached';
      // An expired IMU frame does not erase a still-valid voltage observation.
      // The cached reading is presentation only and is never fed into controls.
      const age=Math.max(0,Date.now()-batteryReading.observedAt)+batteryReading.sourceAge;
      const historical=age>=1500,value=get('battery-percent'),item=get('simple-battery');
      setText(value,batteryReading.value);
      text('battery-voltage',historical?'上次 · '+batteryReading.caption:batteryReading.caption);
      dataset(value,'historical',historical);
      const note=batteryReading.note+(historical?' 上次读取约'+Math.floor(age/1000)+'秒前，当前电压未更新。':'');
      if(value.title!==note)value.title=note;
      text('battery-source-note',note);
      attribute(get('battery-fill').parentElement,'aria-label',historical?'上次读取：'+batteryReading.aria:batteryReading.aria);
      if(item){dataset(item,'state',historical?'historical':'pass');if(item.title!==note)item.title=note;}
      if(historical)text('battery-current-state','保留上次电压');
      window.dispatchEvent(new CustomEvent('duck:battery-display'));
      return;
    }
    const estimated=false;
    const percent=reported;
    const percentText=percent===null?null:percent.toFixed(0);
    const paintKey=[sourceFresh,packFresh,packValid,busValid,reported!==null,percent!==null,b?.source,p?.source,charging].join('|');
    const now=performance.now();
    if(!force&&paintKey===batteryPaintedKey&&now-batteryPaintedAt<100)return;
    batteryPaintedAt=now;batteryPaintedKey=paintKey;
    const value=get('battery-percent');
    setText(value,voltage!==null?voltage.toFixed(busValid&&!packValid?1:2)+' V':'待读取');
    dataset(value,'socKnown',percent!==null);dataset(value,'estimated',estimated);
    dataset(value,'historical',false);
    const fill=get('battery-fill'),width=percent===null?'0%':percentText+'%';
    if(fill.style.width!==width)fill.style.width=width;
    attribute(fill.parentElement,'aria-label',percent===null?'电量未知；仅显示真实电压':'BMS电量 '+percentText+'%');
    text('battery-voltage',reported!==null?'BMS电量 '+percentText+'%':packValid?'电池电压 · 电量未知':busValid?'舵机供电 · 电量未知':'电压待读取');
    text('battery-current-state',voltage!==null?'已读到真实电压':'等待真实电压回包');
    const sourceNote=busValid&&!packValid?'舵机供电电压 '+voltage.toFixed(1)+' V；回包分辨率0.1 V，不代表电池端电压或剩余容量。':
      packValid?'电池传感器电压 '+voltage.toFixed(2)+' V；'+(reported!==null?'SOC由BMS提供。':'没有SOC测量。'):'当前没有有效电压回包；电量和接线状态未知。';
    text('battery-source-note',sourceNote);
    const returned=get('battery-returned');if(returned&&returned.disabled!==!charging)returned.disabled=!charging;
    const item=get('simple-battery');
    if(item){dataset(item,'state',percent!==null||voltage!==null?'pass':'');if(item.title!==sourceNote)item.title=sourceNote;}
    if(value.title!==sourceNote)value.title=sourceNote;
    if(percent!==null||voltage!==null)batteryReading={value:value.textContent,caption:get('battery-voltage').textContent,
      note:sourceNote,aria:fill.parentElement.getAttribute('aria-label'),
      observedAt:servoObserved&&!packValid?Date.now()-p.age_ms:Date.now(),sourceAge:packFresh?b.age_ms:0};
    window.dispatchEvent(new CustomEvent('duck:battery-display'));
  }

  for(const [group,label] of [[0,'左腿'],[1,'右腿'],[2,'头颈']]){
    const title=document.createElement('h3');title.textContent=label;
    const grid=document.createElement('div');grid.className='servo-picker';
    for(const [id,name] of joints.slice(group*5,group*5+5)){
      const button=document.createElement('button');button.type='button';button.dataset.id=String(id);
      button.textContent=String(id);button.title=name;button.setAttribute('aria-label','选择 ID '+id+' '+name);
      button.addEventListener('click',()=>selectJoint(id));grid.append(button);
    }
    get('servo-picker-groups').append(title,grid);
  }
  for(const button of get('workflow-tabs').querySelectorAll('button'))button.addEventListener('click',()=>showPage(button.dataset.page));
  for(const button of document.querySelectorAll('[data-open-page]'))button.addEventListener('click',()=>showPage(button.dataset.openPage));
  get('battery-returned')?.addEventListener('click',()=>{charging=false;try{localStorage.setItem('duck-battery-returned-20261012','yes');}catch{}showBattery(lastStatus,true);});
  window.addEventListener('duck:power-status',event=>{if(event.detail?.status?.power_observation_validated===true)showBattery(event.detail.status);});
  window.addEventListener('duck:live-status',event=>{lastStatus=event.detail?.error||event.detail?.paused?null:event.detail?.status;lastCheckedAt=lastStatus?Date.now():0;renderSelected();showBattery(lastStatus);});
  document.addEventListener('visibilitychange',()=>{if(document.hidden){lastStatus=null;lastCheckedAt=0;wirelessCheckedAt=0;renderSelected();showBattery(null);}});
  document.addEventListener('visibilitychange',syncNumberingFrame);
  document.addEventListener('visibilitychange',syncModelsFrame);
  get('numbering-frame')?.addEventListener('load',syncNumberingFrame);
  window.addEventListener('message',event=>{
    const frame=get('numbering-frame');
    if(frame&&event.source===frame.contentWindow&&event.origin===window.location?.origin&&event.data?.type==='duck-numbering-ready'&&event.data.version===1)syncNumberingFrame();
  });
  setInterval(()=>{if(wirelessCheckedAt&&Date.now()-wirelessCheckedAt>=1800){wirelessCheckedAt=0;renderSelected();}if(lastCheckedAt&&Date.now()-lastCheckedAt>1800){lastStatus=null;lastCheckedAt=0;renderSelected();}if(batteryReading&&(!lastStatus||Date.now()-batteryReading.observedAt+batteryReading.sourceAge>=1500))showBattery(null);},500);
  window.addEventListener('duck:wireless-calibration',event=>{wirelessCalibration=event.detail;wirelessCheckedAt=!document.hidden&&event.detail?.fresh===true?Date.now():0;renderSelected();renderCalibrationProgress();});
  document.addEventListener('DOMContentLoaded',()=>{
    let saved=defaultPage;try{saved=localStorage.getItem(pageKey)||defaultPage;}catch{}
    showPage(saved);showBattery(null,true);
  });
})();
