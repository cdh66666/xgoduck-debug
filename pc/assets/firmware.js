(()=>{
  'use strict';
  const $=id=>document.getElementById(id),dialog=$('firmware-dialog');
  let value=null,timer=null,loading=false;
  const names={queued:'等待更新',preflight:'核对卸力与更新包',stopping_off:'停止旧程序',starting_disabled:'安装并启动，保持卸力',rolling_back:'更新核验失败，正在回退',installed_off_verified:'更新成功 · 已读回核验 · 保持卸力',rolled_back_off:'已恢复旧版 · 保持卸力',preflight_failed:'更新未开始',rollback_failed:'恢复结果待确认'};
  const short=x=>typeof x==='string'?x.slice(0,12):'未知';
  function render(v){
    value=v;const r=v.running,c=v.candidate||{},s=v.state||{},j=v.job;
    $('firmware-live').textContent=!v.connected?'未连接':r?.running_verified?`${r.release} · 已核验`:'旧版未提供完整运行标识';
    $('firmware-mcu').textContent=r?.mcu_build||'未收到 MCU 版本回执';
    $('firmware-python').textContent=short(r?.python_sha256?.['python/controller.py']||v.disk?.['python/controller.py']);
    $('firmware-elf').textContent=short(r?.elf_sha256||v.disk?.['.cache/sketch/sketch.ino.elf']);
    $('firmware-candidate').textContent=c.version||(v.backendPending?'更新接口待就绪':'尚无更新包');
    $('firmware-state').textContent=j?(names[j.phase]||j.phase)+(j.error?'；'+j.error:''):v.error||c.reason||'读取中';
    const same=r?.running_verified&&r.release===c.version;
    $('firmware-update').disabled=loading||v.busy||v.backendPending||!v.connected||!c.ready||s.mode!=='off'||s.enabled!==false||same;
    $('firmware-update').textContent=v.busy?'更新中…':v.backendPending?'更新接口待重启':same?'已是当前版本':'更新到此版本';
    $('firmware-note').textContent=v.busy?'更新期间保持卸力，页面可关闭；重新打开可查看进度。':s.enabled===true?'先停止并确认卸力，再更新。':r?.running_verified?'版本来自运行中的程序与 MCU 回执，更新不会自动恢复动作。':'文件指纹只能说明磁盘文件；收到 MCU 和运行程序回执后才标记“已核验”。';
    if(v.backendPending)$('firmware-note').textContent='板端版本可直接读取。电脑调试服务尚未重启，重启后即可使用一键更新。';
    $('firmware-details').textContent=JSON.stringify({running:r,disk:v.disk,checked_at:v.checked_at,job:j},null,2);
  }
  async function read(){
    if(loading)return;loading=true;$('firmware-refresh').disabled=true;
    try{const r=await fetch('/api/bench/firmware',{cache:'no-store'});
      if(r.status===404){const direct=await fetch('/api/runtime',{cache:'no-store'});if(!direct.ok)throw Error('读取失败');const running=await direct.json();render({connected:true,running,backendPending:true,candidate:{reason:'运行版本已从板端读回'},checked_at:Date.now()/1000});}
      else {if(!r.ok)throw Error('读取失败');render(await r.json());}}
    catch(e){value={...value,connected:false,error:'版本读取失败，请确认电脑调试服务已加载新版接口。'};}
    finally{loading=false;$('firmware-refresh').disabled=false;if(value)render(value);clearTimeout(timer);if(dialog.open&&value?.busy)timer=setTimeout(read,2500);}
  }
  $('firmware-open').onclick=()=>{dialog.showModal();read();};
  $('firmware-close').onclick=()=>dialog.close();dialog.addEventListener('close',()=>clearTimeout(timer));
  $('firmware-refresh').onclick=read;
  $('firmware-update').onclick=async()=>{
    if(loading||!value?.candidate?.ready)return;loading=true;$('firmware-update').disabled=true;
    try{const r=await fetch('/api/bench/firmware/update',{method:'POST',headers:{'Content-Type':'application/json','X-Token':window.COMMISSION_TOKEN},body:JSON.stringify({version:value.candidate.version})});const data=await r.json();if(!r.ok)throw Error(data.detail||'更新未确认');render(data);}
    catch(e){$('firmware-state').textContent=e.message;}
    finally{loading=false;timer=setTimeout(read,1500);}
  };
})();
