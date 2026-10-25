/* Historical data only. This module has no control endpoint or POST request. */
(() => {
  'use strict';
  const finite=Number.isFinite;
  const tilt=s=>Array.isArray(s?.gravity)&&s.gravity.length===3&&s.gravity.every(finite)?
    Math.atan2(Math.hypot(s.gravity[0],s.gravity[1]),-s.gravity[2])*180/Math.PI:null;
  const estimatedTilt=s=>s?.body_attitude_estimate?.basis==='raw_acceleration_only'?tilt({gravity:s.body_attitude_estimate.gravity}):null;
  const phases={walk:'行走模型',getup:'起身模型',default_pose:'准备',off:'卸力',hold:'校准中位',shadow:'只推理'};
  const phaseLabel=s=>s?.sit_down?.active?(s.sit_down.error?'坐姿保护 · 保持支撑':({lowering:'缓慢坐下',seated:'坐姿保持',raising:'缓慢起身',standing:'站姿就绪',settling:'姿态到位中'}[s.sit_down.phase]||'坐下模型')):s?.mode==='policy'?(s.enabled===false?'策略请求 · 尚未上力':phases[s.recovery?.phase]||'策略控制'):
    s?.mode==='legacy'?(s.enabled?'上力 · 旧原始反馈':'已卸力 · 旧原始反馈'):
    phases[s?.mode]||s?.mode||'旧记录 · 模型阶段缺失';
  class Timeline {
    constructor(frames){
      this.rows=frames.filter(row=>finite(row.t)&&row.status&&typeof row.status==='object');
      if(!this.rows.length||this.rows.some((row,i)=>i&&row.t<=this.rows[i-1].t))throw Error('记录时间轴不连续或为空');
      this.first=this.rows[0].t;this.last=this.rows.at(-1).t;
    }
    at(time){
      const t=Math.max(this.first,Math.min(this.last,time));let lo=0,hi=this.rows.length-1;
      while(lo<hi){const mid=Math.ceil((lo+hi)/2);if(this.rows[mid].t<=t)lo=mid;else hi=mid-1;}
      const row=this.rows[lo],next=this.rows[lo+1];
      return {row,index:lo,time:t,gap:t-row.t>.15&&(!next||next.t-row.t>.2)};
    }
  }
  function create({getLive,onMode,postPose,note,canReplay=()=>true}){
    const $=id=>document.getElementById(id),selector=$('motion-history'),bar=$('motion-replay');
    let data=null,timeline=null,time=0,playing=false,request=null,lastTick=0,generation=0,items=[],listKey='',pendingLink=true;
    const setText=(id,value)=>{if($(id).textContent!==value)$(id).textContent=value;};
    const active=()=>timeline!==null;
    const blocked=()=>{
      const s=getLive();return !canReplay()||s?.enabled===true||['policy','hold','calibrate','servo_debug'].includes(s?.mode)||!!s?.factory_calibration?.held_mask;
    };
    function send(){
      if(!timeline)return;
      const sample=timeline.at(time),s=sample.row.status,angle=tilt(s),estimate=estimatedTilt(s),phase=s.recovery?.phase;
      postPose({type:'motion-replay-frame',version:1,id:data.summary.id,index:sample.index,time:sample.row.t,
        status:s,gap:sample.gap,source:data.summary.source,playing,reset:sample.index===0});
      $('motion-seek').value=String(time);
      setText('motion-time',`${time.toFixed(2)} / ${timeline.last.toFixed(2)} s`);
      setText('motion-phase',sample.gap?'缺帧 · 保留上次姿态':phaseLabel(s));
      setText('motion-tilt',angle===null?(estimate===null?'倾角未记录':`估算倾角 ${estimate.toFixed(1)}°`):`倾角 ${angle.toFixed(1)}°`);
      setText('motion-command',Array.isArray(s.command)?'指令 '+s.command.map(v=>Number(v).toFixed(1)).join(' / '):'旧记录 · 指令缺失');
      const error=Array.isArray(s.angles)&&Array.isArray(s.targets)?Math.max(...s.angles.map((v,i)=>Math.abs(v-s.targets[i]))):null;
      setText('motion-detail',[
        `模型：${s.active_model||'旧记录未保存'}；模式：${s.mode||'未知'}；MCU 帧：${s.mcu_seq??'—'}`,
        `与记录期望角的最大差：${finite(error)?error.toFixed(1)+'°':'—'}；反馈年龄：${finite(s.feedback_age_ms)?s.feedback_age_ms.toFixed(1)+' ms':'旧记录未保存'}`,
        `缺失关节：${(s.missing_joint_ids||[]).join(', ')||'无'}；控制错误：${s.last_error??'旧记录未保存'}`,
        '记录期望角不是最终总线 raw goal；全局位置裁剪可能改变实际发送目标。',
        `来源：${data.summary.source_note||'鸭子端原始状态快照'}；帧 ${sample.index+1} / ${timeline.rows.length}`,
        `Walk SHA：${data.summary.provenance?.sha256?.['xgoduck_walk.onnx']||'旧记录未绑定'}`
      ].join('\n'));
      $('motion-play').textContent=playing?'暂停':'播放';
      draw();
    }
    function draw(){
      const canvas=$('motion-timeline'),ctx=canvas.getContext('2d');if(!ctx||!timeline)return;
      const w=canvas.width,h=canvas.height,pad=8,span=Math.max(.001,timeline.last-timeline.first);
      const x=t=>pad+(t-timeline.first)/span*(w-2*pad),y=a=>h-8-a/180*(h-16);
      ctx.clearRect(0,0,w,h);ctx.fillStyle='#0b1824';ctx.fillRect(0,0,w,h);
      ctx.strokeStyle='#354451';ctx.setLineDash([4,5]);ctx.beginPath();ctx.moveTo(pad,y(45));ctx.lineTo(w-pad,y(45));ctx.stroke();ctx.setLineDash([]);
      ctx.strokeStyle=data.summary.source==='legacy-mcu'?'#eab177':'#54c9e8';ctx.setLineDash(data.summary.source==='legacy-mcu'?[4,4]:[]);ctx.lineWidth=2;ctx.beginPath();let previous=null;
      for(const row of timeline.rows){const angle=tilt(row.status)??estimatedTilt(row.status);if(angle===null){previous=null;continue;}
        if(previous&&row.t-previous.t<=.2)ctx.lineTo(x(row.t),y(angle));else ctx.moveTo(x(row.t),y(angle));previous=row;}
      ctx.stroke();ctx.setLineDash([]);
      const warning=data.summary.tilt_after_upright_at_s;
      if(finite(warning)){ctx.fillStyle='#f3aa66';ctx.fillRect(x(warning)-2,0,4,h);}
      ctx.strokeStyle='#f2f5f8';ctx.lineWidth=1.5;ctx.beginPath();ctx.moveTo(x(time),0);ctx.lineTo(x(time),h);ctx.stroke();
    }
    function pause(){playing=false;if(request!==null)cancelAnimationFrame(request);request=null;if(active())send();}
    function tick(now){
      request=null;if(!playing||!timeline)return;
      time=Math.min(timeline.last,time+(now-lastTick)/1000*Number($('motion-speed').value));lastTick=now;
      if(time>=timeline.last)playing=false;send();if(playing)request=requestAnimationFrame(tick);
    }
    function live(){
      ++generation;pause();data=null;timeline=null;selector.value='';bar.hidden=true;
      document.body.dataset.replaying='false';setText('motion-view-title','实时姿态');
      postPose({type:'motion-replay-end',version:1});onMode(false);
    }
    async function select(id){
      if(!id){live();return;}
      if(blocked()){selector.value=active()?data.summary.id:'';note('先停止 / 卸力，再查看历史姿态。');return;}
      const version=++generation;pause();
      try{
        const response=await fetch('/api/bench/motions/'+encodeURIComponent(id),{cache:'no-store',signal:AbortSignal.timeout(10000)});
        if(!response.ok)throw Error('动作还未同步完成');const episode=await response.json();
        if(version!==generation)return;
        if(blocked())throw Error('鸭子正在动作，请先停止 / 卸力');
        timeline=new Timeline(episode.frames);data=episode;time=timeline.first;
        selector.value=id;bar.hidden=false;document.body.dataset.replaying='true';setText('motion-view-title','历史姿态');
        $('motion-seek').min=String(timeline.first);$('motion-seek').max=String(timeline.last);
        $('motion-download').href='/api/bench/motions/'+encodeURIComponent(id)+'/download';
        $('motion-download').download=id+'.jsonl.gz';
        $('motion-source').textContent=episode.summary.source==='legacy-mcu'?'旧原始包 · 关节实测 / 机身估算':'实机记录 · 只读回放';
        $('motion-jump').disabled=!finite(episode.summary.tilt_after_upright_at_s);
        $('motion-jump').title=$('motion-jump').disabled?'没有已记录的站起后大幅倾斜标记':'';
        onMode(true);send();
      }catch(error){if(version===generation){selector.value=active()?data.summary.id:'';note(error.message);}}
    }
    async function refresh(){
      try{
        const response=await fetch('/api/bench/motions',{cache:'no-store',signal:AbortSignal.timeout(3500)});
        if(!response.ok)throw Error('记录列表暂不可用');const report=await response.json();items=report.items||[];
        const completed=items.filter(row=>row.archived&&row.frames>0),key=completed.map(row=>row.id).join('|');
        if(key!==listKey){
          const selected=active()?data.summary.id:'';selector.replaceChildren();
          const option=document.createElement('option');option.value='';option.textContent='实时 · 最新实测';selector.append(option);
          for(const item of completed){const node=document.createElement('option');node.value=item.id;
            const date=new Date(item.start_unix_s*1000),stamp=date.toLocaleString('zh-CN',{month:'2-digit',day:'2-digit',hour:'2-digit',minute:'2-digit',second:'2-digit',hour12:false});
            node.textContent=`${stamp} · ${item.label||'动作'} · ${Number(item.duration_s||0).toFixed(1)} s`;
            selector.append(node);}
          selector.value=selected;listKey=key;
        }
        const running=items.find(row=>row.state==='recording'),recorder=report.recorder;
        const synced=finite(report.last_sync_unix_s)&&Date.now()/1000-report.last_sync_unix_s<5;
        setText('motion-recording-state',running&&synced?'● 记录中':!report.error&&synced&&recorder?.sampler_alive&&recorder?.writer_alive&&!recorder?.errors?'自动记录已就绪':completed.length?'已保存 '+completed.length+' 次 · 记录器待连接':'记录器待连接');
        $('motion-recording-state').title=report.error||recorder?.error||'鸭子端25Hz自动记录，电脑保存完整历史；鸭子端缓存16MiB。';
        if(pendingLink){pendingLink=false;const params=new URLSearchParams(location.search),id=params.get('replay');if(id&&completed.some(row=>row.id===id)){await select(id);const at=params.get('t');if(timeline&&at!==null&&finite(Number(at))){time=Math.max(timeline.first,Math.min(timeline.last,Number(at)));send();}}}
      }catch{setText('motion-recording-state','历史可离线查看 · 记录器待连接');}
    }
    selector.onchange=()=>select(selector.value);
    $('motion-live').onclick=live;
    $('motion-play').onclick=()=>{if(!timeline)return;if(playing){pause();return;}if(time>=timeline.last)time=timeline.first;playing=true;lastTick=performance.now();send();request=requestAnimationFrame(tick);};
    $('motion-seek').oninput=()=>{const value=Number($('motion-seek').value);pause();time=value;send();};
    for(const [id,step] of [['motion-prev',-1],['motion-next',1]])$(id).onclick=()=>{if(!timeline)return;pause();const current=timeline.at(time).index;time=timeline.rows[Math.max(0,Math.min(timeline.rows.length-1,current+step))].t;send();};
    $('motion-timeline').onclick=event=>{if(!timeline)return;pause();const rect=event.currentTarget.getBoundingClientRect();time=timeline.first+Math.max(0,Math.min(1,(event.clientX-rect.left)/rect.width))*(timeline.last-timeline.first);send();};
    $('motion-jump').onclick=()=>{if(!timeline)return;pause();time=Math.max(timeline.first,(data.summary.tilt_after_upright_at_s??timeline.first)-1);send();};
    document.addEventListener('visibilitychange',()=>{if(document.hidden)pause();});
    refresh();setInterval(refresh,2000);
    return {get active(){return active();},pause,redraw:send,select,live,update(){selector.disabled=blocked();}};
  }
  window.DuckMotionHistory={Timeline,tilt,estimatedTilt,phaseLabel,create};
})();
