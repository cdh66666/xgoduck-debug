/* Sequential numbering, official API. Never sends goto or mode commands. */
(() => {
  'use strict';const $=id=>document.getElementById(id),IDS=[10,11,12,13,14,20,21,22,23,24,30,31,32,33,34];
  let active=false,owned=false,busy=false,completed=new Set(),visible=true,cancelled=false,stage='idle',taskKind='';
  const note=value=>$('status').textContent=value;
  async function api(path,body){const r=await fetch('/api/'+path,{cache:'no-store',signal:AbortSignal.timeout(4000),...(body===undefined?{}:{method:'POST',headers:{'Content-Type':'application/json','X-Token':window.NUMBERING_TOKEN},body:JSON.stringify(body)})});const value=await r.json();if(!r.ok)throw Error(value.detail||'请求未确认');return value;}
  const servo=(action,id,extra={})=>api('servo',{action,target_id:id,...extra});
  function nextId(){return IDS.find(id=>!completed.has(id));}
  function paint(){
    for(const id of ['maintenance-enter','maintenance-exit','port'])$(id).disabled=busy;
    $('numbering-target').disabled=true;
    const next=nextId();$('numbering-target').value=next??'';$('target-id').textContent=next??'已完成';$('completed-count').textContent=completed.size+' / 15';
    $('scan').disabled=!active||busy&&taskKind!=='scan';$('start').disabled=!active||busy&&taskKind!=='number';
    $('scan').textContent=busy&&taskKind==='scan'?'停止扫描':'扫描 ID';
    $('start').textContent=busy&&taskKind==='number'?(stage==='scanning'?'停止扫描 / 等待':'停止后续 / 完成读回'):'自动编号';
  }
  function stop(){cancelled=true;note(stage==='scanning'?'正在停止；当前读取结束后不再扫描':'本次写入已开始，将完成目标与原 ID 读回，不再执行后续操作');}
  function checkStop(){if(cancelled)throw Error('扫描已停止，未开始编号写入');}
  async function task(kind,fn){if(busy){if(taskKind===kind)stop();return;}busy=true;taskKind=kind;cancelled=false;stage='scanning';paint();try{await fn();}catch(e){note(e.message+(cancelled&&stage==='scanning'?'':'；未自动重试'));}finally{busy=false;stage='idle';taskKind='';paint();}}
  function phase(id,state){$(id).dataset.state=state;}
  for(const id of IDS){const option=document.createElement('option');option.value=id;option.textContent='ID '+id;$('numbering-target').append(option);}
  $('maintenance-enter').onclick=()=>task('enter',async()=>{await api('servo/enter',{});owned=true;for(let i=0;i<5;i++){const s=await api('status');active=s.mode==='servo_debug'&&s.mcu_servo_debug===true;if(active)break;}if(!active){owned=false;throw Error('舵机维护未确认');}note('舵机维护已进入；只改号，不归中');});
  $('maintenance-exit').onclick=()=>task('exit',async()=>{await api('servo/exit',{});active=owned=false;note('舵机维护已退出');});
  async function scan(){
    const found=[];
    for(const id of [1,...IDS]){checkStop();note('扫描 ID '+id);const r=await servo('ping',id);if(r.target_id!==id||typeof r.ok!=='boolean')throw Error('扫描回复不匹配');if(r.ok)found.push(id);checkStop();}
    completed=new Set(found.filter(id=>IDS.includes(id)));
    const selected=$('port').value||'1';$('port').replaceChildren();
    for(const id of [1,...found.filter(id=>id!==1)]){const option=document.createElement('option');option.value=id;option.textContent='ID '+id+(id===1?' · 待编号':'');$('port').append(option);}
    $('port').value=found.includes(+selected)?selected:'1';
    $('feedback').textContent=found.map(id=>'ID '+id+' 在线').join('\n');paint();
    note('已编号 '+completed.size+' / 15 · '+(nextId()?'下一颗 ID '+nextId():'标准编号已齐'));
    return found;
  }
  $('scan').onclick=()=>task('scan',scan);
  $('start').onclick=()=>task('number',async()=>{
    const source=Number($('port').value||1);
    if(!Number.isInteger(source)||source<0||source>253)throw Error('当前 ID 无效');
    for(const id of ['phase-connect','phase-numbering','phase-centering','phase-release'])phase(id,'pending');
    phase('phase-connect','active');const found=await scan();checkStop();
    if(!found.includes(source))throw Error('当前 ID '+source+' 未应答');
    const target=nextId();if(target===undefined)throw Error('15个标准编号已齐，无需改号');
    phase('phase-connect','done');phase('phase-numbering','active');stage='writing';paint();
    let writeError=null;try{await servo('set_id',source,{new_id:target});}catch(e){writeError=e;}
    // Once a write has started, STOP never skips either readback, including an
    // uncertain write acknowledgement. No automatic repeated EEPROM write.
    phase('phase-numbering',writeError?'error':'done');phase('phase-centering','active');stage='verifying';paint();
    let read=null,old=null;const errors=[];
    try{read=await servo('read',target);}catch(e){errors.push(e.message);}
    try{old=await servo('ping',source);}catch(e){errors.push(e.message);}
    if(!Number.isFinite(read?.raw_pos)||old?.target_id!==source||old?.ok!==false)throw Error('ID '+source+' → '+target+' 改号读回未确认，请先扫描'+(errors.length?' · '+errors.join('；'):''));
    phase('phase-numbering','done');phase('phase-centering','done');phase('phase-release','done');completed.delete(source);completed.add(target);
    $('port').value='1';$('feedback').textContent=`ID ${source} → ${target} · 编码 ${read.raw_pos} · 未发送位置目标`;
    note('编号已读回确认'+(writeError?'（写入回执超时，实际读回成功）':'')+' · 已编号 '+completed.size+' / 15 · '+(nextId()?'下一颗 ID '+nextId():'标准编号已齐')+(cancelled?' · 已停止后续':''));
  });
  async function poll(){if(busy||document.hidden||!visible)return;try{const s=await api('status');active=owned&&s.mode==='servo_debug'&&s.mcu_servo_debug===true;if(owned&&s.mode!=='servo_debug')owned=false;if(active)await api('servo/enter',{});paint();}catch{active=owned=false;paint();}}
  window.addEventListener('message',e=>{if(e.origin===location.origin&&e.source===parent&&e.data?.type==='duck-numbering-visibility'){visible=e.data.visible===true;if(visible)poll();}});
  note('点击进入舵机维护后操作。打开页面不改变模式。');paint();poll();setInterval(poll,1000);
})();
