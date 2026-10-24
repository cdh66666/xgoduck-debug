/* Display only. Space real poses on the robot's snapshot clock, not Wi-Fi bursts. */
import {DelayedPosePlayback} from './pose-playback.js';

const finite=Number.isFinite;
const uint32=value=>Number.isInteger(value)&&value>=0&&value<=0xffffffff;
const identity=source=>Number.isInteger(source?.pc_host_epoch)&&source.pc_host_epoch>=0?source.pc_host_epoch:null;
const angles=source=>(source?.native_full_robot_ready??source?.full_robot_ready)===true&&Array.isArray(source.angles)&&
  source.angles.length===15&&source.angles.every(finite)?source.angles.slice():null;

export function officialDisplayHeading(source){
  const heading=source?.display_heading;
  return heading?.available===true&&finite(heading.deg)&&['current','held'].includes(heading.quality)&&
    heading.reference==='mcu-power-on-full-pose'&&heading.relative===true&&heading.drift_expected===true&&
    /^[0-9a-f]{16}$/.test(heading.boot_epoch)&&uint32(heading.source_seq)&&uint32(heading.source_imu_us)&&
    Number.isInteger(heading.segment)&&heading.segment>=0&&typeof heading.continuous==='boolean'&&
    finite(heading.age_ms)&&heading.age_ms>=0?{...heading}:null;
}

export class OfficialPosePlayback extends DelayedPosePlayback {
  constructor(options={}){
    super(options);
    this.clock=options.clock||'robot-uptime';
    this.acceptedEpoch=null;this.acceptedSequence=null;this.acceptedObservedMs=null;this.acceptedSourceMs=null;
  }
  reset(reason){super.reset(reason);this.headingRows=[];this.headingWatermark=null;}
  endpoint(source){
    const epoch=identity(source),observedMs=source?.pc_observed_monotonic_s*1000;
    const sourceMs=this.clock==='pc-observation'?observedMs:source?.uptime_s*1000;
    return epoch!==null&&uint32(source?.mcu_seq)&&finite(observedMs)&&observedMs>0&&finite(sourceMs)&&sourceMs>=0?
      {epoch,observedMs,sourceMs,sequence:source.mcu_seq}:null;
  }
  advances(endpoint){
    if(!endpoint||this.retiredEpochs.has(endpoint.epoch))return false;
    if(this.acceptedEpoch===null||endpoint.epoch!==this.acceptedEpoch)return true;
    const step=(endpoint.sequence-this.acceptedSequence)>>>0;
    return step>0&&step<0x80000000&&endpoint.observedMs>this.acceptedObservedMs&&endpoint.sourceMs>this.acceptedSourceMs;
  }
  remember(endpoint){
    if(this.acceptedEpoch!==null&&endpoint.epoch!==this.acceptedEpoch){
      this.retiredEpochs.add(this.acceptedEpoch);
      if(this.retiredEpochs.size>64)this.retiredEpochs.delete(this.retiredEpochs.values().next().value);
    }
    this.acceptedEpoch=endpoint.epoch;this.acceptedSequence=endpoint.sequence;
    this.acceptedObservedMs=endpoint.observedMs;
    this.acceptedSourceMs=endpoint.sourceMs;
  }
  // While hidden, consume only a watermark. Returning never plays hidden backlog.
  discard(source){const endpoint=this.endpoint(source);if(this.advances(endpoint))this.remember(endpoint);}
  setPaused(value){
    const wasPaused=this.paused;super.setPaused(value);
    if(wasPaused&&!this.paused)this.awaitingResume=true;
  }
  observeBatch(current,frames,poseOf,receivedAtMs){
    // Live display has no history queue. A radio burst must not replay its
    // oldest endpoint or permanently anchor the display to a late first frame.
    if(this.delayMs===0){
      const pose=poseOf(current),accepted=!!pose&&this.observe(pose.quaternion,current,receivedAtMs);
      return {accepted,currentAccepted:accepted};
    }
    let accepted=false,currentAccepted=false;
    const history=this.epoch===null||this.awaitingResume?[]:Array.isArray(frames)?frames:[];
    for(const source of [...history,current]){
      const pose=poseOf(source);
      if(!pose||!this.observe(pose.quaternion,source,receivedAtMs))continue;
      accepted=true;
      if(source.mcu_seq===current?.mcu_seq&&source.pc_host_epoch===current?.pc_host_epoch&&
         source.pc_observed_monotonic_s===current?.pc_observed_monotonic_s)currentAccepted=true;
    }
    return {accepted,currentAccepted};
  }
  observe(pose,source,receivedAtMs){
    if(this.paused)return false;
    const endpoint=this.endpoint(source),q=Array.isArray(pose)?pose:pose?.quaternion;
    // Endpoint validity belongs to its native observation. Delivery age only
    // affects current controls/labels; delayed real poses remain in the buffer.
    const age=source?.native_feedback_age_ms??source?.feedback_age_ms,norm=Array.isArray(q)?Math.hypot(...q):0;
    const knownAge=source?.native_feedback_age_ms==null?
      age+(source?.pc_delivery_age_lower_bound_ms??0):source?.feedback_age_ms;
    if(!this.advances(endpoint)||!Array.isArray(q)||q.length!==4||!q.every(finite)||norm<1e-8||
      !finite(receivedAtMs)||!finite(age)||age<0||age>=150||!finite(knownAge)||knownAge<0)return false;
    if(this.lastObservedAtMs!==null&&receivedAtMs<this.lastObservedAtMs)return false;
    if(this.acceptedEpoch!==null&&endpoint.epoch!==this.acceptedEpoch)this.reset('pc-host-epoch-changed');
    if(this.epoch===null){
      this.epoch=endpoint.epoch;this.hostClockOriginMs=endpoint.sourceMs;
      this.originAtMs=receivedAtMs;this.scheduleClock=this.clock;
    }
    const row={sourceMs:endpoint.sourceMs,observedMs:endpoint.observedMs,timelineMs:endpoint.sourceMs-this.hostClockOriginMs,
      receivedAtMs,ageMs:knownAge,quaternion:q.map(value=>value/norm),sequence:endpoint.sequence,
      epoch:endpoint.epoch,segment:endpoint.epoch,sourceUs:null,angles:angles(source)};
    this.rows.push(row);this.sourceMs=row.sourceMs;this.timelineMs=row.timelineMs;
    const heading=officialDisplayHeading(source),previous=this.headingWatermark;
    if(heading){
      const sameSegment=previous&&heading.boot_epoch===previous.boot_epoch&&heading.segment===previous.segment;
      const step=previous?(heading.source_seq-previous.source_seq)>>>0:1;
      // A cached 20 Hz heading inside a newer 100 Hz robot snapshot is not a
      // second heading observation and cannot advance its presentation clock.
      if(!sameSegment||step>0&&step<0x80000000){
        this.headingRows.push({sourceMs:row.sourceMs,heading});this.headingWatermark=heading;
      }
    }else if(this.headingRows.at(-1)?.heading!==null)this.headingRows.push({sourceMs:row.sourceMs,heading:null});
    const earliest=row.sourceMs-this.delayMs-this.maxGapMs-1000;
    while(this.rows.length>2&&this.rows[1].sourceMs<earliest)this.rows.shift();
    if(this.rows.length>this.maxSamples)this.rows.splice(0,this.rows.length-this.maxSamples);
    while(this.headingRows.length>2&&this.headingRows[1].sourceMs<earliest)this.headingRows.shift();
    if(this.headingRows.length>this.maxSamples)this.headingRows.splice(0,this.headingRows.length-this.maxSamples);
    if(!this.held)this.held={quaternion:row.quaternion.slice(),sourceMs:row.sourceMs,left:row,right:row,fraction:0};
    this.sequence=endpoint.sequence;this.lastObservedAtMs=receivedAtMs;this.remember(endpoint);
    this.awaitingResume=false;
    return true;
  }
  frame(now,state,reason){
    const frame=super.frame(now,state,reason),held=this.held,left=held?.left,right=held?.right;
    const joints=left?.angles&&right?.angles?left.angles.map((value,index)=>value+(right.angles[index]-value)*held.fraction):null;
    let heading=null,headingQuality='unknown',headingIdentity=null,headingSourceSequences=null;
    const target=held?.sourceMs;
    let index=-1;
    while(index+1<this.headingRows.length&&this.headingRows[index+1].sourceMs<=target)index++;
    const a=this.headingRows[index],b=this.headingRows[index+1],h=a?.heading,next=b?.heading;
    if(h){
      heading=h.deg;headingIdentity=h.boot_epoch+':'+h.segment;
      headingQuality=target===a.sourceMs&&h.quality==='current'?'current':'historical';
      headingSourceSequences=[h.source_seq,h.source_seq];
      if(next&&h.quality==='current'&&next.quality==='current'&&h.continuous&&next.continuous&&
         h.boot_epoch===next.boot_epoch&&h.segment===next.segment&&b.sourceMs-a.sourceMs<=this.maxGapMs){
        const fraction=(target-a.sourceMs)/(b.sourceMs-a.sourceMs);
        heading=h.deg+(next.deg-h.deg)*fraction;headingQuality='current';
        headingSourceSequences=[h.source_seq,next.source_seq];
      }
    }
    return {...frame,jointAnglesDegrees:joints,endpointObservedMs:held?[left.observedMs,right.observedMs]:null,
      endpointSourceUs:null,headingAvailable:heading!==null,headingDeg:heading,headingQuality,
      headingIdentity,headingSourceSequences};
  }
}

