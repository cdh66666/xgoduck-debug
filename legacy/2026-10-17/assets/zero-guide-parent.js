/* Model reference selection only; no raw position or actuator writes. */
(() => {
  const frame=document.getElementById('zero-guide-frame');
  if(!frame)return;
  const ids=[10,11,12,13,14,20,21,22,23,24,30,31,32,33,34],groups=['left','right','head'];
  let sent=null,ready=false,selectedId=10,desired=null;
  function selected(){const id=window.DuckWorkflow?.selectedJoint();if(ids.includes(id))selectedId=id;return selectedId;}
  function sync(){
    if(!ready)return;
    const id=selected(),key=desired?.type==='group'?'group:'+desired.group:'joint:'+id;
    if(key===sent)return;
    if(desired?.type==='group')frame.contentWindow.postMessage({type:'zero-guide-group',group:desired.group},location.origin);
    else if(ids.includes(id))frame.contentWindow.postMessage({type:'zero-guide-select',id},location.origin);
    sent=key;
  }
  window.addEventListener('message',event=>{
    if(event.origin!==location.origin||event.source!==frame.contentWindow)return;
    if(event.data?.type==='zero-guide-ready'){
      ready=true;frame.dataset.loaded='true';const id=selected(),group=window.duckWirelessCalibration?.view().group;
      if(!desired&&groups.includes(group))desired={type:'group',group};
      sent=desired?.type==='group'&&event.data.view==='group'&&event.data.group===desired.group?'group:'+desired.group:
        !desired&&event.data.view==='whole'&&event.data.id===id?'joint:'+id:null;
      sync();
    }
    if(event.data?.type==='zero-guide-joint-selected'&&ids.includes(event.data.id)){
      desired={type:'joint'};
      if(window.DuckWorkflow?.selectJoint)window.DuckWorkflow.selectJoint(event.data.id);
      else document.querySelector('#servo-picker-groups button[data-id="'+event.data.id+'"]')?.click();
      selectedId=event.data.id;sent='joint:'+event.data.id;
    }
  });
  window.addEventListener('duck:page',sync);
  window.addEventListener('duck:wireless-calibration',sync);
  window.addEventListener('duck:calibration-group',event=>{if(groups.includes(event.detail?.group)){desired={type:'group',group:event.detail.group};sent=null;sync();}});
  window.addEventListener('duck:joint',event=>{if(ids.includes(event.detail?.id)){selectedId=event.detail.id;desired={type:'joint'};sent=null;sync();}});
  setInterval(sync,300);
})();
