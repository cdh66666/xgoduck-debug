(function(scope){
  'use strict';
  const IDS=[10,11,12,13,14,20,21,22,23,24,30,31,32,33,34];
  const GROUPS={left:IDS.slice(0,5),right:IDS.slice(5,10),head:IDS.slice(10)};
  const NAMES=['左髋转向','左髋侧摆','左髋前后摆','左膝','左踝','右髋转向','右髋侧摆','右髋前后摆','右膝','右踝','颈俯仰','头俯仰','头转向','头侧倾','嘴巴'];
  const CONTRACT=2,PASSIVE_CONTRACT=1,TOLERANCE=57,LINK_LEASE_MS=1000;
  const validIds=value=>Array.isArray(value)&&new Set(value).size===value.length&&value.every(id=>IDS.includes(id));
  const canonical=value=>Array.isArray(value)?value.map(canonical):value&&typeof value==='object'?
    Object.fromEntries(Object.keys(value).sort().map(key=>[key,canonical(value[key])])):value;
  function createSequenceController(options){
    const now=options.now||Date.now,changed=options.change||(()=>{});
    let envelope=null,observedAt=0,session=null,ownerBoot=null,selectedId=null,busy=false,pending=false,stopping=false,stopPending=false;
    let hidden=false,page='imu',generation=0,lastAck=0,lastSent=-Infinity,inflight=0,message='读取校准状态';
    function view(){
      const status=envelope?.status,sequence=status?.sequence_calibration,elapsed=now()-observedAt;
      const supported=sequence?.supported===true&&sequence.contract_revision===CONTRACT&&sequence.tolerance_ticks===TOLERANCE;
      const fresh=envelope?.connection?.online===true&&status?.fresh===true&&elapsed>=0&&elapsed<LINK_LEASE_MS&&sequence?.fresh===true;
      const completed=validIds(sequence?.completed_ids)?sequence.completed_ids:[],held=validIds(sequence?.held_ids)?sequence.held_ids:[];
      const rows=new Map((status?.joints||[]).map(row=>[row.id,row]));
      const allOff=fresh&&status.all_torque_off_verified===true&&status.torque_may_be_on===false&&sequence?.active===false;
      const passive=status?.passive_id10,id10=rows.get(10);
      const verifiedHash=value=>typeof value==='string'&&/^[0-9a-f]{64}$/.test(value);
      const canPassiveId10=false; // Keep the original auxiliary button inactive; normal Seq2 capture handles every ID.
      const states=Object.fromEntries(IDS.map(id=>{
        const row=rows.get(id),read=fresh&&row?.fresh===true&&Number.isFinite(row.metadata_age_ms)&&row.metadata_age_ms<=750;
        const state=completed.includes(id)?read&&held.includes(id)&&sequence.active===true&&sequence.holding_verified===true&&row.torque_enabled===true?'holding':read&&row.torque_enabled===false?'off':'recorded':'pending';
        return [id,state];
      }));
      const needsStop=!!session||pending||stopping||sequence?.active===true;
      const id10CaptureBlocked=false;
      const canCapture=!hidden&&page==='imu'&&!busy&&!stopping&&supported&&envelope?.connection?.online===true&&
        envelope.connection.update_interlocked!==true&&IDS.includes(selectedId)&&!id10CaptureBlocked&&
        (!sequence.active||!!session);
      const scope=sequence?.scope_ids,offIds=sequence?.off_verified_ids;
      const scopeOff=supported&&sequence?.active===false&&sequence.release_verified===true&&
        validIds(scope)&&validIds(offIds)&&scope.every(id=>offIds.includes(id));
      return {status,sequence,selectedId,busy,supported,fresh,scopeOff,completed:completed.slice(),held:held.slice(),states,allOff,needsStop,canCapture,canPassiveId10,id10CaptureBlocked,owned:!!session,message};
    }
    const notify=()=>changed(view());
    function accept(value){
      const previous=envelope?.status,next=value?.status;
      if(previous&&next&&previous.boot_id===next.boot_id&&Number.isInteger(previous.sequence)&&Number.isInteger(next.sequence)){
        const step=(next.sequence-previous.sequence)>>>0;if(step>=0x80000000)return;
      }
      envelope=value;observedAt=now();
      if(session&&(next?.boot_id!==ownerBoot||value?.connection?.update_interlocked===true)){void stop('校准连接已改变；正在卸力。');return;}
      if(stopping&&view().scopeOff){stopping=false;message='已停止，本次输出轴卸力已核对。';}
      notify();
    }
    async function stop(reason='正在停止并卸力…',explicit=false){
      const fallback=explicit&&!view().supported;
      const token=session;generation++;session=null;ownerBoot=null;pending=false;busy=false;stopping=true;message=reason;notify();
      if(stopPending)return false;stopPending=true;
      try{
        const result=await options.request(fallback?'off_all':'sequence_stop',fallback?{supported:true}:{id:0,sequence_contract_revision:CONTRACT,...(token?{session:token}:{})},{keepalive:true});
        if(result?.status)accept({connection:{online:true},status:result.status});
        if(result?.ok===true&&(fallback?view().allOff:view().scopeOff)){stopping=false;message=fallback?'已停止、全机卸力已核对。':'已停止，本次输出轴卸力已核对。';return true;}
        message='停止待确认，请保持托稳。';return false;
      }catch{message='停止回执未确认，请保持托稳。';return false;}
      finally{stopPending=false;notify();}
    }
    function tick(){
      if(session&&(hidden||page!=='imu'||now()-lastAck>=LINK_LEASE_MS))void stop('保持反馈中断；正在卸力。');
      else notify();
    }
    async function heartbeat(){
      tick();if(!session||hidden||page!=='imu'||now()-lastSent<100||inflight>=4)return;
      const token=session,mark=generation;lastSent=now();inflight++;
      try{
        const result=await options.request('sequence_heartbeat',{id:0,session:token,sequence_contract_revision:CONTRACT});
        if(mark!==generation||token!==session)return;
        const sequence=result?.status?.sequence_calibration;
        if(result?.ok!==true||result.op!=='sequence_heartbeat'||now()-lastAck>=LINK_LEASE_MS||result.status.boot_id!==ownerBoot||
          sequence?.contract_revision!==CONTRACT||sequence.active!==true)throw Error('heartbeat_unconfirmed');
        lastAck=now();accept({connection:{online:true},status:result.status});
      }catch(error){if(mark===generation&&token===session&&(error.result||now()-lastAck>=LINK_LEASE_MS))void stop('保持回执中断；正在卸力。');}
      finally{inflight--;}
    }
    async function capture(){
      if(!view().canCapture)return false;
      const id=selectedId,mark=generation;busy=true;message='正在核对当前关节…';notify();
      try{
        const fresh=await options.readStatus({signal:AbortSignal.timeout(1500)});
        if(mark!==generation||hidden||page!=='imu')return false;accept(fresh);
        const check=view(),status=check.status,sequence=check.sequence;
        if(!check.supported||!check.fresh||fresh.connection?.update_interlocked===true)throw Error('等待新鲜校准反馈。');
        if(!session){
          if(status.armed?.active===true||status.session_active===true||status.operation)throw Error('已有其它控制会话，请先停止该会话。');
          pending=true;
          const begun=await options.request('sequence_begin',{id:0,supported:true,clear:true,sequence_contract_revision:CONTRACT});
          if(mark!==generation||hidden||page!=='imu'){
            if(typeof begun?.session==='string')void options.request('sequence_stop',{id:0,session:begun.session,sequence_contract_revision:CONTRACT},{keepalive:true});
            return false;
          }
          const begunSequence=begun?.status?.sequence_calibration;
          if(begun?.ok!==true||begun.op!=='sequence_begin'||typeof begun.session!=='string'||!begun.session||begunSequence?.active!==true||begunSequence.contract_revision!==CONTRACT||begun.status.boot_id!==status.boot_id)throw Error('校准会话未确认。');
          session=begun.session;ownerBoot=status.boot_id;pending=false;lastAck=now();lastSent=-Infinity;
          accept({connection:{online:true},status:begun.status});void heartbeat();
        }
        if(mark!==generation||!session)return false;
        const selected=await options.request('sequence_select',{id,session,sequence_contract_revision:CONTRACT});
        if(mark!==generation||!session)return false;
        if(selected?.ok!==true||selected.op!=='sequence_select'||selected.status?.sequence_calibration?.selected_id!==id)throw Error('所选关节未确认。');
        accept({connection:{online:true},status:selected.status});if(mark!==generation||!session)return false;message='记录 ID '+id+' 零位并等待保持确认…';notify();
        const captured=await options.request('sequence_capture_hold',{id,session,pose_confirmed:true,supported:true,sequence_contract_revision:CONTRACT});
        if(mark!==generation||!session)return false;
        if(captured?.status)accept({connection:{online:true},status:captured.status});
        const verified=view();
        if(captured?.ok!==true||captured.op!=='sequence_capture_hold'||!verified.completed.includes(id)||verified.states[id]!=='holding')throw Error('记录或保持尚未确认。');
        message=verified.completed.length===15?'15 颗已记录并保持；可按停止卸力。':'ID '+id+' 已记录并保持，继续选择下一颗。';
        return true;
      }catch(error){
        if(mark===generation){
          const reason=typeof options.formatError==='function'&&error.result?options.formatError(error.result):error.message||'校准未完成。';
          if(session||pending)await stop(reason+' 正在卸力。');else message=reason;
        }
        return false;
      }finally{if(mark===generation)busy=false;notify();}
    }
    function select(id){if(busy||id!==null&&!IDS.includes(id))return false;selectedId=id;notify();return true;}
    async function passiveCapture(confirmStep){
      const original=view();
      if(!original.canPassiveId10||typeof confirmStep!=='function')return false;
      if(await confirmStep(1,original)!==true||await confirmStep(2,original)!==true)return false;
      const current=view();
      if(!current.canPassiveId10||current.status?.boot_id!==original.status?.boot_id||
          current.status?.passive_id10?.expected_native_sha256!==original.status?.passive_id10?.expected_native_sha256)return false;
      busy=true;message='正在被动记录 ID10 中位；不会给舵机上力…';notify();
      try{
        const passive=current.status.passive_id10;
        const saved=await options.request('passive_id10_record',{
          id:10,passive_contract_revision:PASSIVE_CONTRACT,id10_at_neutral:true,supported:true,
          leg_suspended:true,clearance_checked:true,
          expected_group_sha256:passive.expected_group_sha256,
          expected_native_sha256:passive.expected_native_sha256});
        if(saved?.ok!==true||saved.op!=='passive_id10_record'||saved.id!==10||saved.torque_writes!==0||saved.goal_writes!==0||
            saved.mcu_request_seq_unchanged!==true||saved.powered_hold_verified!==false)throw Error('被动记录回执不完整。');
        const after=await options.readStatus();accept(after);
        const beforeRecords=current.status.records,afterRecords=after?.status?.records;
        const other14=IDS.filter(id=>id!==10).every(id=>beforeRecords?.[String(id)]&&afterRecords?.[String(id)]&&
          JSON.stringify(canonical(beforeRecords[String(id)]))===JSON.stringify(canonical(afterRecords[String(id)])));
        if(after?.status?.group_file?.sha256!==saved.group_sha256||
            after.status?.native_file?.sha256!==saved.native_sha256||
            after.status?.native_file?.group_records_sha256!==saved.group_sha256||
            afterRecords?.['10']?.source!=='passive-torque-off-DWR1'||
            afterRecords?.['10']?.position!==saved.native_position||!other14)
          throw Error('文件可能已写入，但新旧零位核验未完成；不要重复点击。');
        message='ID10 中位已被动记录，仍需带电保持与动作验收。';return true;
      }catch(error){message=error?.message||'ID10 被动记录未确认；不要自动重试。';return false;}
      finally{busy=false;notify();}
    }
    function visibility(value){hidden=!!value;if(hidden&&(session||pending||stopping))void stop('页面已隐藏；正在卸力。');else notify();}
    function setPage(value){page=value;if(page!=='imu'&&(session||pending||stopping))void stop('已离开校准页面；正在卸力。');else notify();}
    return {view,accept,select,capture,passiveCapture,heartbeat,tick,stop,visibility,setPage};
  }
  if(typeof module==='object'&&module.exports){module.exports={IDS,GROUPS,CONTRACT,TOLERANCE,createSequenceController};return;}
  const get=id=>document.getElementById(id),wireless=scope.DuckWirelessCalibration;
  if(!get('sequence-panel')||!wireless)return;
  let opened=false,highlightIds=[],selectedGroup=null;
  const request=wireless.createRequest({fetch:(...args)=>fetch(...args),getToken:()=>scope.COMMISSION_TOKEN,setToken:value=>{scope.COMMISSION_TOKEN=value;}});
  const readStatus=async extra=>{const response=await fetch(wireless.STATUS_URL,{cache:'no-store',signal:extra?.signal});if(!response.ok)throw Error('无线未连接。');return response.json();};
  function highlight(){
    const frame=get('imu-duck-frame');frame?.contentWindow?.postMessage({type:'imu-model-highlight',version:1,ids:opened?highlightIds:[]},location.origin);
  }
  function setOpen(value){
    opened=!!value;get('sequence-panel').hidden=!opened;get('pose-calibration-layout').dataset.calibrating=String(opened);
    get('sequence-toggle').setAttribute('aria-expanded',String(opened));get('wl-home-state').hidden=opened;get('wl-result').hidden=opened;highlight();
  }
  function select(id){
    if(!controller.select(id))return;
    highlightIds=[id];selectedGroup=null;highlight();render(controller.view());
  }
  function render(view){
    get('sequence-capture').disabled=!view.canCapture;
    const passiveButton=get('sequence-passive-id10');if(passiveButton)passiveButton.disabled=!view.canPassiveId10;
    get('sequence-capture').title='记录所选关节的当前原生位置并上力保持；其余已保持关节继续保持。';
    get('sequence-count').textContent=view.completed.length+' / 15';
    get('sequence-selected').textContent=IDS.includes(view.selectedId)?'ID '+view.selectedId+' · '+NAMES[IDS.indexOf(view.selectedId)]:'选择一颗舵机';
    get('sequence-result').textContent=!view.supported?'校准程序待更新（Seq2）':view.message;
    for(const button of get('sequence-joints').querySelectorAll('button')){
      const id=Number(button.dataset.id),state=view.states[id];button.dataset.state=state;button.disabled=view.busy;
      button.setAttribute('aria-pressed',String(highlightIds.includes(id)));
      button.querySelector('small').textContent={holding:'保持',off:'卸力',recorded:'已录',pending:'待录'}[state];
      button.title='ID '+id+' '+NAMES[IDS.indexOf(id)]+' · '+{holding:'新鲜反馈已确认保持',off:'已记录，当前已卸力',recorded:'已记录，保持待确认',pending:'本轮尚未记录'}[state];
    }
    for(const button of document.querySelectorAll('[data-sequence-group]')){button.disabled=view.busy;button.setAttribute('aria-pressed',String(button.dataset.sequenceGroup===selectedGroup));}
    if(view.needsStop)get('wl-home').disabled=true;
  }
  let lastStatus=null;
  const controller=createSequenceController({request,readStatus,formatError:wireless.formatError,change:view=>{
    render(view);if(view.status&&view.status!==lastStatus){lastStatus=view.status;scope.dispatchEvent(new CustomEvent('duck:sequence-status',{detail:{status:view.status}}));}
  }});
  scope.duckSequenceCalibration={needsStop:()=>controller.view().needsStop,stop:()=>controller.stop(undefined,true),view:controller.view};
  for(const id of IDS){
    const button=document.createElement('button');button.type='button';button.dataset.id=String(id);button.className='quiet';button.setAttribute('aria-label','校准 ID '+id+' '+NAMES[IDS.indexOf(id)]);
    const label=document.createElement('span'),state=document.createElement('small');label.textContent=String(id);state.textContent='待录';button.append(label,state);button.addEventListener('click',()=>select(id));get('sequence-joints').append(button);
  }
  for(const button of document.querySelectorAll('[data-sequence-group]'))button.addEventListener('click',()=>{
    if(controller.view().busy)return;controller.select(null);selectedGroup=button.dataset.sequenceGroup;highlightIds=GROUPS[selectedGroup].slice();highlight();render(controller.view());
  });
  get('sequence-toggle').addEventListener('click',()=>setOpen(!opened));
  get('sequence-capture').addEventListener('click',event=>{if(event.isTrusted)void controller.capture();});
  get('sequence-passive-id10')?.addEventListener('click',event=>{
    if(!event.isTrusted)return;
    void controller.passiveCapture(step=>step===1?
      scope.confirm('确认 ID10 已手动摆到机械中位、机身托稳、左腿悬空且线束有余量？'):
      scope.confirm('仅被动记录 ID10 软件零位；不向舵机发送上力或目标。原文件有备份，继续吗？'));
  });
  scope.addEventListener('duck:calibration-panel',event=>setOpen(event.detail?.open===true));
  scope.addEventListener('duck:wireless-calibration',event=>{if(event.detail?.status){const current=scope.duckWirelessCalibration?.view();controller.accept({connection:{online:event.detail.fresh===true||current?.connected===true,update_interlocked:current?.interlocked===true},status:event.detail.status});}});
  scope.addEventListener('duck:page',event=>controller.setPage(event.detail.page));
  scope.addEventListener('message',event=>{
    const frame=get('imu-duck-frame');if(event.origin!==location.origin||event.source!==frame?.contentWindow)return;
    if(event.data?.type==='imu-model-ready')highlight();
    if(opened&&event.data?.type==='imu-model-joint-selected'&&event.data.version===1&&IDS.includes(event.data.id))select(event.data.id);
  });
  document.addEventListener('visibilitychange',()=>controller.visibility(document.hidden));
  scope.addEventListener('pagehide',()=>controller.visibility(true));
  setInterval(()=>{controller.tick();void controller.heartbeat();},100);
  render(controller.view());
})(typeof window==='object'?window:globalThis);
