/* Historical presentation only: real endpoints, bounded delay, no actuator API. */
const uint32=value=>Number.isInteger(value)&&value>=0&&value<=0xffffffff;
const finite=Number.isFinite;
const quaternion=value=>{
  const q=Array.isArray(value)?value:value?.quaternion;
  if(!Array.isArray(q)||q.length!==4||!q.every(finite))return null;
  const norm=Math.hypot(...q);
  return norm>1e-8?q.map(component=>component/norm):null;
};

export function interpolateObservedQuaternion(left,right,fraction){
  const a=quaternion(left),b=quaternion(right);
  if(!a||!b||!finite(fraction)||fraction<0||fraction>1)return null;
  let dot=a.reduce((sum,value,index)=>sum+value*b[index],0),target=b;
  if(dot<0){dot=-dot;target=b.map(value=>-value);}
  dot=Math.min(1,dot);
  const angle=Math.acos(dot),sine=Math.sin(angle);
  const weights=sine<1e-6?[1-fraction,fraction]:
    [Math.sin((1-fraction)*angle)/sine,Math.sin(fraction*angle)/sine];
  return quaternion(a.map((value,index)=>value*weights[0]+target[index]*weights[1]));
}

/**
 * Feed only accepted sensor observations, using the same monotonic browser clock
 * for observe() and step(). The600ms label is an intentional playback schedule,
 * not a measurement of total sensor/network latency. Original source freshness
 * and control permissions belong to the caller and must remain unchanged.
 *
 * step() is a20FPS output gate; it never catches up by emitting frame bursts.
 * `playing` includes historical interpolation and must never mean live telemetry.
 * On underflow or a source gap>300ms, the last presentation freezes explicitly.
 */
export class DelayedPosePlayback {
  constructor({fps=20,delayMs=600,maxGapMs=300,maxSamples=256}={}){
    if(!finite(fps)||fps<1||fps>60||!finite(delayMs)||delayMs<0||delayMs>2000||
       !finite(maxGapMs)||maxGapMs<=0||maxGapMs>2000||!Number.isInteger(maxSamples)||maxSamples<2||maxSamples>4096)
      throw new RangeError('Invalid bounded playback options');
    this.fps=fps;this.frameMs=1000/fps;this.delayMs=delayMs;
    this.maxGapMs=maxGapMs;this.maxSamples=maxSamples;
    this.paused=false;this.retiredEpochs=new Set();this.reset();
  }

  reset(reason='reset'){
    this.rows=[];this.epoch=null;this.sourceUs=null;this.sequence=null;
    this.sourceMs=0;this.timelineMs=0;this.hostClockOriginMs=null;this.scheduleClock=null;
    this.originAtMs=null;this.lastObservedAtMs=null;
    this.nextFrameAtMs=null;this.lastStepAtMs=null;this.held=null;
    this.hasPlayed=false;this.state=this.paused?'paused':'buffering';this.reason=reason;
  }

  setPaused(value){
    const paused=value===true;
    if(paused===this.paused)return;
    // A hidden page never accumulates or replays a queue on return. Retain only
    // its last still image, clearly historical, while the new buffer fills.
    const held=this.held;
    this.paused=paused;this.reset(paused?'paused':'resumed');this.held=held;
  }

  observe(pose,source,receivedAtMs){
    if(this.paused)return false;
    const q=quaternion(pose),age=source?.feedback_age_ms;
    if(!q||!finite(receivedAtMs)||!uint32(source?.mcu_seq)||!uint32(source?.mcu_imu_us)||
       !uint32(source?.mcu_restarts)||typeof source?.app_instance_id!=='string'||
       !source.app_instance_id||source.app_instance_id.length>128||!finite(age)||age<0)return false;
    // Historical display accepts real endpoints at their known age; this grants no control freshness.
    const epoch=source.app_instance_id+':'+source.mcu_restarts;
    if(this.retiredEpochs.has(epoch))return false;
    const knownEpoch=this.epoch??this.held?.left.epoch??null;
    if(knownEpoch!==null&&epoch!==knownEpoch){
      this.retiredEpochs.add(knownEpoch);
      if(this.retiredEpochs.size>64)this.retiredEpochs.delete(this.retiredEpochs.values().next().value);
      this.reset('source-epoch-changed');
    }
    if(this.lastObservedAtMs!==null&&receivedAtMs<this.lastObservedAtMs)return false;
    const delta=this.sourceUs===null?0:(source.mcu_imu_us-this.sourceUs)>>>0;
    const sequenceStep=this.sequence===null?1:(source.mcu_seq-this.sequence)>>>0;
    if(this.sourceUs!==null&&(delta===0||delta>=0x80000000||sequenceStep===0||sequenceStep>=0x80000000))return false;
    // Host observation time is a display-only clock. MCU micros() can drift
    // from the browser by >1ms/s; treating those clocks as identical consumes
    // the fixed600ms buffer even while every source sample arrives.
    // Choose one clock for this epoch. Missing/regressed Host metadata then
    // freezes presentation until it advances again; never rebase after stalls.
    const hostMs=source.display_observed_monotonic_ms,hasHostClock=finite(hostMs)&&hostMs>0;
    let timelineMs;
    if(this.epoch===null){
      this.scheduleClock=hasHostClock?'host-monotonic':'mcu-fallback';
      this.hostClockOriginMs=hasHostClock?hostMs:null;
      timelineMs=0;
    }else if(this.scheduleClock==='host-monotonic'){
      if(!hasHostClock)return false;
      timelineMs=hostMs-this.hostClockOriginMs;
      if(!finite(timelineMs)||timelineMs<=this.timelineMs||
        Math.abs(timelineMs-this.timelineMs-delta/1000)>Math.max(100,delta/1000*.02))return false;
    }else timelineMs=this.timelineMs+delta/1000;
    if(this.epoch===null){this.epoch=epoch;this.originAtMs=receivedAtMs;}
    this.sourceMs+=delta/1000;this.timelineMs=timelineMs;
    const row={sourceMs:this.sourceMs,timelineMs,receivedAtMs,ageMs:age,quaternion:q,
      sourceUs:source.mcu_imu_us,sequence:source.mcu_seq,epoch,segment:Number.isInteger(source.display_segment)&&source.display_segment>=0?source.display_segment:null};
    this.rows.push(row);
    const earliest=this.sourceMs-this.delayMs-this.maxGapMs-1000;
    while(this.rows.length>2&&this.rows[1].sourceMs<earliest)this.rows.shift();
    if(this.rows.length>this.maxSamples)this.rows.splice(0,this.rows.length-this.maxSamples);
    if(!this.held)this.held={quaternion:q.slice(),sourceMs:row.sourceMs,left:row,right:row,fraction:0};
    this.sourceUs=source.mcu_imu_us;this.sequence=source.mcu_seq;this.lastObservedAtMs=receivedAtMs;
    return true;
  }

  frame(now,state,reason){
    this.state=state;this.reason=reason;
    const held=this.held;
    return {quaternion:held?.quaternion.slice()||null,state,reason,
      fps:this.fps,nominalDelayMs:this.delayMs,historical:true,currentTelemetry:false,
      interpolated:!!held&&held.fraction>0&&held.fraction<1,
      sourceTimeMs:held?.sourceMs??null,sourceEpoch:held?.left.epoch??this.epoch,sourceSegment:held?.left.segment??null,
      sourceSequences:held?[held.left.sequence,held.right.sequence]:null,
      endpointSourceUs:held?[held.left.sourceUs,held.right.sourceUs]:null,
      knownEndpointAgesMs:held?[Math.max(0,now-held.left.receivedAtMs)+held.left.ageMs,
        Math.max(0,now-held.right.receivedAtMs)+held.right.ageMs]:null,
      bufferAheadMs:this.rows.length&&this.originAtMs!==null?
        this.rows.at(-1).timelineMs-(now-this.originAtMs-this.delayMs):null,
      scheduleClock:this.scheduleClock,renderedAtMs:now};
  }

  step(now){
    if(this.paused||!finite(now))return null;
    if(this.lastStepAtMs!==null&&now<this.lastStepAtMs)return null;
    this.lastStepAtMs=now;
    if(this.nextFrameAtMs!==null&&now+1e-8<this.nextFrameAtMs)return null;
    // A delayed callback never allows a second frame a few milliseconds later.
    // Callers should schedule the next callback at nextFrameAtMs (50ms default).
    this.nextFrameAtMs=now+this.frameMs;
    if(!this.rows.length||this.originAtMs===null)return this.frame(now,'buffering','waiting-for-observations');
    const target=now-this.originAtMs-this.delayMs;
    if(target<this.rows[0].timelineMs)return this.frame(now,'buffering','filling-delay-buffer');
    let leftIndex=0;
    while(leftIndex+1<this.rows.length&&this.rows[leftIndex+1].timelineMs<=target+1e-8)leftIndex++;
    const left=this.rows[leftIndex],right=this.rows[leftIndex+1];
    if(Math.abs(target-left.timelineMs)<1e-8){
      this.held={quaternion:left.quaternion.slice(),sourceMs:left.sourceMs,left,right:left,fraction:0};
    }else if(!right){
      return this.frame(now,this.hasPlayed?'underflow':'buffering','no-observed-future-endpoint');
    }else if(left.segment!==null&&right.segment!==null&&left.segment!==right.segment){
      return this.frame(now,'gap','observation-segment-changed');
    }else if(right.sourceMs-left.sourceMs>this.maxGapMs){
      return this.frame(now,'gap','source-gap-exceeds-interpolation-bound');
    }else{
      const fraction=(target-left.timelineMs)/(right.timelineMs-left.timelineMs);
      this.held={quaternion:interpolateObservedQuaternion(left.quaternion,right.quaternion,fraction),
        sourceMs:left.sourceMs+(right.sourceMs-left.sourceMs)*fraction,left,right,fraction};
    }
    if(leftIndex>0)this.rows.splice(0,leftIndex);
    this.hasPlayed=true;
    return this.frame(now,'playing','delayed-observed-presentation');
  }
}
