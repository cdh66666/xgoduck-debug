/* Diagram interactions choose a joint only; they never send a movement target. */
(() => {
  'use strict';
  const get=id=>document.getElementById(id);
  const ids=[10,11,12,13,14,20,21,22,23,24,30,31,32,33,34];
  let lastLiveAt=null,selectedId=10,lastDetail={paused:true},networkKey='',paintedJoint=null,paintedBattery=null;
  const hotspots=[...document.querySelectorAll('.joint-hotspot')],meter=get('battery-percent'),batterySymbol=document.querySelector('.battery-symbol');
  const setText=(element,value)=>{if(element&&element.textContent!==value)element.textContent=value;};
  const networkVisible=()=>!document.hidden&&window.DuckWorkflow?.page()==='link';
  const fresh=s=>s&&Number.isFinite(s.received)&&s.received>0&&Number.isInteger(s.mcu_seq)&&s.mcu_seq>=0&&s.mcu_seq<=4294967295&&Number.isFinite(s.feedback_age_ms)&&s.feedback_age_ms>=0&&s.feedback_age_ms<150;
  function color(id,state){const element=get(id);if(element&&element.dataset.state!==state)element.dataset.state=state;}
  function network(detail,force=false,observe=true){
    const s=detail?.status,live=!detail?.error&&!detail?.paused&&fresh(s)&&
      (observe||lastLiveAt!==null&&performance.now()-lastLiveAt<=1800);
    if(observe){lastDetail=detail;lastLiveAt=live?performance.now():null;}
    if(!networkVisible()&&!force)return;
    const ids=live&&Array.isArray(s.servo_ids)?new Set(s.servo_ids.filter(id=>Number.isInteger(id)&&[10,11,12,13,14,20,21,22,23,24,30,31,32,33,34].includes(id))):null;
    const imu=live&&s.imu_ok===true;
    const key=[live,ids?.size??'unknown',imu,detail?.paused===true].join('|');
    if(!force&&key===networkKey)return;networkKey=key;
    for(const id of ['network-pc','network-board','network-wifi'])color(id,live?'ok':'unknown');
    for(const id of ['network-imu','network-imu-wire'])color(id,imu?'ok':'unknown');
    for(const id of ['network-servos','network-servo-wire'])color(id,ids&&ids.size===15?'ok':ids&&ids.size?'partial':'unknown');
    setText(get('network-servo-count'),ids?String(ids.size):'—');
    setText(get('visual-network-state'),live?'已连接':detail?.paused?'暂停':'离线');
    setText(get('visual-network-note'),live?(ids?.size||0)+' / 15 · '+(imu?'IMU 在线':'IMU 待确认'):'等待主控');
  }
  function selection(event){
    const eventId=event?.detail?.id;
    if(ids.includes(eventId))selectedId=eventId;
    const reported=window.DuckWorkflow?.selectedJoint();
    if(ids.includes(reported))selectedId=reported;
    const value=selectedId;
    if(paintedJoint!==value){paintedJoint=value;hotspots.forEach(element=>{const pressed=String(Number(element.dataset.joint)===value);if(element.getAttribute('aria-pressed')!==pressed)element.setAttribute('aria-pressed',pressed);});}
    syncBattery();
  }
  function syncBattery(){
    const text=meter?.textContent||'';
    const known=meter?.dataset?.socKnown==='true'||/^约?\s*\d+(?:\.\d+)?%$/.test(text);
    if(paintedBattery!==known){paintedBattery=known;batterySymbol?.classList.toggle('unknown',!known);}
  }
  for(const element of hotspots){
    const choose=()=>{const id=Number(element.dataset.joint);if(window.DuckWorkflow?.selectJoint)window.DuckWorkflow.selectJoint(id);else get('servo-picker-groups')?.querySelector('button[data-id="'+id+'"]')?.click();selection();};
    element.addEventListener('click',choose);
    element.addEventListener('keydown',event=>{if(event.key==='Enter'||event.key===' '){event.preventDefault();choose();}});
  }
  window.addEventListener('duck:live-status',event=>network(event.detail));
  window.addEventListener('duck:page',()=>{selection();if(networkVisible())network(lastDetail,true,false);});
  window.addEventListener('duck:joint',selection);
  window.addEventListener('duck:battery-display',syncBattery);
  document.addEventListener('visibilitychange',()=>{if(document.hidden)network({paused:true});});
  setInterval(()=>{if(lastLiveAt!==null&&performance.now()-lastLiveAt>1800)network({error:true});},500);
  network({paused:true},true);selection();
})();
