/* Display-only six-axis relative heading. No I/O, device API, or bias learning.
 * Quaternion [x,y,z,w] maps body -> world, whose gravity is [0,0,-1].
 * Input gravity/gyroRad come from the existing GravityKalman; gyroRad is rad/s
 * with its existing bias compensation. Gravity cannot measure heading or its
 * gyro bias. Slow real yaw is therefore retained, and heading can drift.
 */
const vector = value => Array.isArray(value) && value.length === 3 && value.every(Number.isFinite);
const uint32 = value => Number.isInteger(value) && value >= 0 && value <= 0xffffffff;
const dot = (a,b) => a.reduce((sum,value,i) => sum + value*b[i],0);
const cross = (a,b) => [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const unit = value => {const length=Math.hypot(...value);return Number.isFinite(length)&&length>1e-12?value.map(v=>v/length):null;};
const multiply = (a,b) => {
  const [x,y,z,w]=a,[X,Y,Z,W]=b;
  return [w*X+x*W+y*Z-z*Y,w*Y-x*Z+y*W+z*X,w*Z+x*Y-y*X+z*W,w*W-x*X-y*Y-z*Z];
};
const rotate = (q,v) => {
  const t=cross(q.slice(0,3),v).map(value=>2*value),u=cross(q.slice(0,3),t);
  return v.map((value,i)=>value+q[3]*t[i]+u[i]);
};
const yawRotation = angle => [0,0,Math.sin(angle/2),Math.cos(angle/2)];
const heading = q => {
  const forward=rotate(q,[1,0,0]);
  // Standard yaw is undefined when the body forward axis is vertical.
  if(Math.hypot(forward[0],forward[1])<.01)return null;
  const value=Math.atan2(forward[1],forward[0]);
  return Math.abs(value)<1e-12?0:value;
};
const tilt = g => unit([-g[1],g[0],0,1-g[2]])||[1,0,0,0];
const integrate = (q,omega,dt) => {
  const speed=Math.hypot(...omega);
  if(speed<1e-12||dt===0)return q.slice();
  const half=speed*dt/2,factor=Math.sin(half)/speed;
  return unit(multiply(q,[...omega.map(value=>value*factor),Math.cos(half)]));
};
const alignGravity = (q,g) => {
  const measured=unit(rotate(q,g)),down=[0,0,-1],cosine=Math.max(-1,Math.min(1,dot(measured,down)));
  // Left-side minimum world-space correction has no world-Z rotation axis.
  // It is only a tilt observation; it never supplies a north/yaw observation.
  const correction=unit([...cross(measured,down),1+cosine])||[1,0,0,0];
  return unit(multiply(correction,q));
};
const keepHeading = (q,previous) => {
  const current=heading(q);
  return previous===null||current===null?q:unit(multiply(yawRotation(previous-current),q));
};

export class RelativeHeading {
  constructor(){this.reset();}
  reset(){
    this.q=null;this.us=null;this.sequence=null;this.gyro=null;this.instance=null;this.restarts=null;
    this.lastAt=null;this.mode='waiting';this.breakPending=null;this.discontinuity=null;
    this.continuous=true;this.reference='startup';this.segment=0;
  }
  /** source is the real status {mcu_imu_us,app_instance_id,mcu_restarts}.
   * nowMs is performance.now(), used for freshness/prediction, not integration.
   * Duplicate MCU timestamps never extend freshness or integrate twice.
   */
  observe(pose,source,nowMs){
    const gravity=vector(pose?.gravity)?unit(pose.gravity):null;
    if(!gravity||!vector(pose?.gyroRad)||!Number.isFinite(Math.hypot(...pose.gyroRad))||
       !uint32(source?.mcu_imu_us)||!uint32(source?.mcu_restarts)||
       (source.mcu_seq!==undefined&&!uint32(source.mcu_seq))||
       typeof source?.app_instance_id!=='string'||!source.app_instance_id||!Number.isFinite(nowMs)){
      this.hold('invalid-observation');return this.sample(Number.isFinite(nowMs)?nowMs:this.lastAt);
    }
    const identityChanged=this.q!==null&&(this.instance!==source.app_instance_id||this.restarts!==source.mcu_restarts);
    if(!this.q||identityChanged){
      this.q=keepHeading(tilt(gravity),0);
      this.continuous=!identityChanged;this.discontinuity=identityChanged?'source-restarted':null;
      this.reference=identityChanged?'restart':'startup';this.segment++;
    }else{
      const step=(source.mcu_imu_us-this.us)>>>0;
      if(step===0)return this.sample(nowMs);
      const sequenceStep=this.sequence===null||source.mcu_seq===undefined?1:(source.mcu_seq-this.sequence)>>>0;
      const reason=this.breakPending||(step>=0x80000000?'timestamp-reset':step>100000?'source-gap':
        sequenceStep!==1?'missing-samples':
        nowMs<this.lastAt?'render-clock-reset':nowMs-this.lastAt>=150?'stream-timeout':
        pose.gyroCalibrationChanged===true?'gyro-calibration-changed':
        pose.gravityReacquired===true?'gravity-reacquired':null);
      if(reason){
        // A gyro endpoint says nothing about a missing interval. Apply only the
        // shortest observed gravity correction and keep continuity broken.
        // Holding projected Euler heading near a vertical forward axis would
        // amplify tiny gravity noise into a large real world-Z rotation.
        this.q=alignGravity(this.q,gravity);
        this.continuous=false;this.discontinuity=reason;this.segment++;
      }else{
        const omega=pose.gyroRad.map((value,i)=>(value+this.gyro[i])*.5);
        this.q=alignGravity(integrate(this.q,omega,step/1e6),gravity);
      }
    }
    this.us=source.mcu_imu_us;this.sequence=source.mcu_seq??null;this.gyro=pose.gyroRad.slice();
    this.instance=source.app_instance_id;this.restarts=source.mcu_restarts;
    this.lastAt=nowMs;this.mode='live';this.breakPending=null;
    return {...this.sample(nowMs),changed:true};
  }
  /** Frozen/missing observations never create an integration interval. */
  hold(reason='stream-paused'){
    this.mode=this.q?'stale':'waiting';this.breakPending=String(reason);
    if(this.q){this.continuous=false;this.discontinuity=this.breakPending;}
    return this.sample(this.lastAt);
  }
  /** Prediction is display-only, <=40ms, and never commits into this.q/us. */
  sample(nowMs){
    if(!this.q)return {changed:false,mode:'waiting',quaternion:null,headingDeg:null,projectionObservable:false,
      relative:true,absoluteHeading:false,driftExpected:true,continuous:false,discontinuity:this.discontinuity,
      reference:this.reference,predictionMs:0,sourceUs:null,segment:this.segment};
    if(!Number.isFinite(nowMs))return null;
    const age=nowMs-this.lastAt;
    if(this.mode==='live'&&(age>=150||age<0)){
      this.mode='stale';this.breakPending=age<0?'render-clock-reset':'stream-timeout';
      this.continuous=false;this.discontinuity=this.breakPending;
    }
    const predictionMs=this.mode==='live'?Math.max(0,Math.min(40,age)):0;
    const quaternion=integrate(this.q,this.gyro,predictionMs/1000),angle=heading(quaternion);
    return {changed:false,mode:this.mode,quaternion,headingDeg:angle===null?null:angle*180/Math.PI,
      projectionObservable:angle!==null,relative:true,absoluteHeading:false,driftExpected:true,
      continuous:this.continuous,discontinuity:this.discontinuity,reference:this.reference,
      predictionMs,sourceUs:this.us,segment:this.segment};
  }
  /** Establish a new display reference only. Does not change gravity or bias. */
  zero(nowMs){
    const shown=this.sample(nowMs);
    if(!shown||shown.mode!=='live'||shown.headingDeg===null)return null;
    this.q=unit(multiply(yawRotation(-shown.headingDeg*Math.PI/180),this.q));
    this.continuous=true;this.discontinuity=null;this.reference='user-zero';this.segment++;
    return this.sample(nowMs);
  }
}
