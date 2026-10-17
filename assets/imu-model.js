/* Observed gravity plus display-only relative gyro heading. No device-control API. */
const vector = v => Array.isArray(v) && v.length === 3 && v.every(Number.isFinite);
const uint32 = value => Number.isInteger(value) && value >= 0 && value <= 0xffffffff;
export const unwrapAngle = (angle, previous) => Number.isFinite(angle)&&Number.isFinite(previous)
  ? previous+((angle-previous+180)%360+360)%360-180 : angle;
export function deriveAttitude(acc, previousRoll = 0) {
  if (!vector(acc)) return null;
  const [x,y,z] = acc, norm = Math.hypot(x,y,z), transverse = Math.hypot(y,z);
  if (!Number.isFinite(norm) || norm < .1) return null;
  const deg = 180 / Math.PI, rollObservable = transverse / norm > .01;
  return {roll:rollObservable ? Math.atan2(-y,-z)*deg : Number.isFinite(previousRoll)?previousRoll:0,
    pitch:Math.atan2(x,transverse)*deg, tilt:Math.atan2(Math.hypot(x,y),-z)*deg,
    rollObservable, norm, gravity:acc.map(v=>v/norm), yaw:null, yawObservable:false};
}

/* Six-state gravity/bias EKF. Yaw remains unobservable and is never integrated. */
export class GravityKalman {
  constructor(){this.reset();}
  reset(){this.v=null;this.bias=[0,0,0];this.P=null;this.us=null;this.instance=null;this.gyro=null;this.lastPose=null;this.gravityConflict=null;}
  static cross(a,b){return [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];}
  static skew(v){return [[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]];}
  static multiply(a,b){return a.map(row=>b[0].map((_,j)=>row.reduce((sum,value,k)=>sum+value*b[k][j],0)));}
  static transpose(a){return a[0].map((_,i)=>a.map(row=>row[i]));}
  static inverse(a){
    const [[b,c,d],[e,f,g],[h,i,j]]=a;
    const determinant=b*(f*j-g*i)-c*(e*j-g*h)+d*(e*i-f*h);
    if(!Number.isFinite(determinant)||Math.abs(determinant)<1e-12)return null;
    return [[f*j-g*i,d*i-c*j,c*g-d*f],[g*h-e*j,b*j-d*h,d*e-b*g],[e*i-f*h,c*h-b*i,b*f-c*e]].map(row=>row.map(v=>v/determinant));
  }
  static propagateGravity(v,omega,dt){
    const speed=Math.hypot(...omega),angle=speed*dt;
    const axis=speed>1e-12?omega.map(value=>value/speed):[0,0,0],cross=GravityKalman.cross(axis,v);
    const dot=axis.reduce((sum,value,i)=>sum+value*v[i],0),co=Math.cos(angle),si=Math.sin(angle);
    return v.map((value,i)=>value*co-cross[i]*si+axis[i]*dot*(1-co));
  }
  reacquireGravity(s,unit,norm,R,radians,reason){
    this.v=unit;
    this.P=this.P.map((row,i)=>row.map((value,j)=>i<3&&j<3?(i===j?R:0):i<3||j<3?0:value));
    this.us=s.mcu_imu_us;this.instance=s.app_instance_id;this.gyro=s.gyro.slice();
    this.reacquirePending=false;this.gravityConflict=null;
    this.lastPose={...deriveAttitude(this.v,this.lastPose?.roll||0),norm,fused:true,
      biasDeg:this.bias.map(value=>value/radians),gyroRad:s.gyro.map((value,i)=>value*radians-this.bias[i]),
      accelerationTrusted:true,gravityReacquired:true,gravityReacquiredReason:reason};
    return this.lastPose;
  }
  consume(s){
    if(!vector(s?.acc)||!vector(s?.gyro)||!Number.isInteger(s.mcu_imu_us)||s.mcu_imu_us<0||s.mcu_imu_us>0xffffffff){this.gravityConflict=null;return null;}
    const norm=Math.hypot(...s.acc),unit=s.acc.map(v=>v/norm),radians=Math.PI/180;
    if(!Number.isFinite(norm)||norm<.1){this.gravityConflict=null;return null;}
    const step=this.us===null?0:(s.mcu_imu_us-this.us)>>>0;
    if(this.v&&this.instance===s.app_instance_id&&step===0)return this.lastPose;
    if(!this.v||this.instance!==s.app_instance_id||step===0||step>=0x80000000){
      this.v=unit;this.bias=[0,0,0];
      this.P=Array.from({length:6},(_,i)=>Array.from({length:6},(_,j)=>i===j?(i<3?(5*radians)**2:(10*radians)**2):0));
      this.us=s.mcu_imu_us;this.instance=s.app_instance_id;this.gyro=s.gyro.slice();this.reacquirePending=false;this.gravityConflict=null;
      this.lastPose={...deriveAttitude(this.v,this.lastPose?.roll||0),norm,fused:true,biasDeg:[0,0,0],gyroRad:s.gyro.map(v=>v*radians),accelerationTrusted:Math.abs(norm-9.80665)<1.96};
      return this.lastPose;
    }
    const accelerationTrusted=norm>7.84532&&norm<11.76798;
    // An endpoint gyro value cannot describe motion in the missing interval.
    // Rebase gravity from the first new usable observation without learning
    // that missing rotation as gyro bias; retain the learned bias covariance.
    if(step>100000)this.reacquirePending=true;
    const gapGravityObservation=this.reacquirePending&&accelerationTrusted;
    if(gapGravityObservation){
      const R=(2*radians)**2*(1+((norm-9.80665)/.4903325)**2);
      const S=this.P.slice(0,3).map((row,i)=>row.slice(0,3).map((value,j)=>value+(i===j?R:0)));
      const inverse=GravityKalman.inverse(S),innovation=unit.map((value,i)=>value-this.v[i]);
      const weighted=inverse?GravityKalman.multiply(inverse,innovation.map(value=>[value])):null;
      const distance=weighted?innovation.reduce((sum,value,i)=>sum+value*weighted[i][0],0):Infinity;
      this.reacquirePending=false;
      // A two-sigma change is distinct from the established static1–2deg noise.
      // Small gap innovations still receive the ordinary gravity smoothing.
      if(distance>4)return this.reacquireGravity(s,unit,norm,R,radians,'missing-interval');
    }
    // An unusable gap endpoint stays held; later adjacent gyro samples remain observed.
    const dt=step<=100000?step/1e6:0,previous=this.v.slice();
    const omega=s.gyro.map((v,i)=>(v+this.gyro[i])*.5*radians-this.bias[i]);
    this.v=GravityKalman.propagateGravity(previous,omega,dt);
    const O=GravityKalman.skew(omega),V=GravityKalman.skew(previous);
    const F=Array.from({length:6},(_,i)=>Array.from({length:6},(_,j)=>
      i<3&&j<3?(i===j?1:0)-dt*O[i][j]:i<3&&j>=3?-dt*V[i][j-3]:i===j?1:0));
    this.P=GravityKalman.multiply(GravityKalman.multiply(F,this.P),GravityKalman.transpose(F));
    for(let i=0;i<6;i++)this.P[i][i]+=(i<3?radians*radians:(.04*radians)**2)*Math.min(step/1e6,.1);
    if(accelerationTrusted){
      const R=(2*radians)**2*(1+((norm-9.80665)/.4903325)**2);
      const S=this.P.slice(0,3).map((row,i)=>row.slice(0,3).map((v,j)=>v+(i===j?R:0))),inverse=GravityKalman.inverse(S);
      if(inverse){
        const innovation=unit.map((v,i)=>v-this.v[i]);
        const weighted=GravityKalman.multiply(inverse,innovation.map(v=>[v]));
        const distance=innovation.reduce((sum,v,i)=>sum+v*weighted[i][0],0);
        if(distance<25){
          this.gravityConflict=null;
          const K=GravityKalman.multiply(this.P.map(row=>row.slice(0,3)),inverse).map((row,i)=>gapGravityObservation&&i>=3?[0,0,0]:row);
          const correction=K.map(row=>row.reduce((sum,v,i)=>sum+v*innovation[i],0));
          this.v=this.v.map((v,i)=>v+correction[i]);this.bias=this.bias.map((v,i)=>v+correction[i+3]);
          const A=Array.from({length:6},(_,i)=>Array.from({length:6},(_,j)=>(i===j?1:0)-(j<3?K[i][j]:0)));
          const joseph=GravityKalman.multiply(GravityKalman.multiply(A,this.P),GravityKalman.transpose(A));
          this.P=joseph.map((row,i)=>row.map((v,j)=>v+R*K[i].reduce((sum,k,n)=>sum+k*K[j][n],0)));
        }else if(Number.isFinite(distance)&&Math.abs(norm-9.80665)<=.4903325){
          // A short missing movement can leave a permanent innovation rejection.
          // Confirm only that rejected gravity: >=4 distinct observations across
          // >=60ms, near1g, and within3deg of one gyro-propagated reference.
          // Ordinary observed motion has no confirmation delay or fixed buffer.
          let conflict=this.gravityConflict;
          const predicted=conflict?GravityKalman.propagateGravity(conflict.gravity,omega,dt):null;
          const agreement=predicted?predicted.reduce((sum,value,i)=>sum+value*unit[i],0):-1;
          if(!conflict||step>100000||agreement<Math.cos(3*radians)){
            conflict={sinceUs:s.mcu_imu_us,count:1,gravity:unit.slice()};
          }else{conflict.count++;conflict.gravity=predicted;}
          this.gravityConflict=conflict;
          if(conflict.count>=4&&((s.mcu_imu_us-conflict.sinceUs)>>>0)>=60000){
            return this.reacquireGravity(s,unit,norm,R,radians,'persistent-gravity-conflict');
          }
        }else{
          this.gravityConflict=null;
        }
      }else this.gravityConflict=null;
    }else this.gravityConflict=null;
    const length=Math.hypot(...this.v);
    if(!Number.isFinite(length)||length<.1||!this.bias.every(Number.isFinite)){this.reset();return null;}
    this.v=this.v.map(v=>v/length);this.us=s.mcu_imu_us;this.instance=s.app_instance_id;this.gyro=s.gyro.slice();
    this.lastPose={...deriveAttitude(this.v,this.lastPose?.roll||0),norm,fused:true,
      biasDeg:this.bias.map(v=>v/radians),gyroRad:s.gyro.map((v,i)=>v*radians-this.bias[i]),accelerationTrusted};
    return this.lastPose;
  }
}

/* Explicit user-confirmed, finite-lived display reference. Never auto-learns
 * stillness and never modifies raw telemetry or any device configuration. */
export class ConfirmedStationaryGyroBias {
  constructor(profile){
    const p=profile;
    this.profile=p?.version===1&&p.scope==='display-only'&&p.user_confirmed_stationary===true&&p.units==='deg/s'&&
      typeof p.id==='string'&&/^[a-f0-9]{64}$/.test(p.id)&&typeof p.app_instance_id==='string'&&!!p.app_instance_id&&
      uint32(p.mcu_restarts)&&vector(p.bias_deg_s)&&p.bias_deg_s.every(v=>Math.abs(v)<=20)&&
      vector(p.std_deg_s)&&p.std_deg_s.every(v=>v>=0&&v<.35)&&vector(p.heldout_residual_deg_s)&&p.heldout_residual_deg_s.every(v=>Math.abs(v)<.1)&&
      Number.isInteger(p.samples)&&p.samples>=200&&Number.isFinite(p.duration_ms)&&p.duration_ms>=10000&&
      Number.isFinite(p.created_at_ms)&&p.created_at_ms>0&&Number.isFinite(p.expires_at_ms)&&
      p.expires_at_ms>p.created_at_ms&&p.expires_at_ms-p.created_at_ms<=1800000
      ?Object.freeze({...p,bias_deg_s:Object.freeze(p.bias_deg_s.slice())}):null;
  }
  correct(source,wallNow){
    const p=this.profile;
    const active=!!(p&&Number.isFinite(wallNow)&&wallNow>=p.created_at_ms&&wallNow<p.expires_at_ms&&
      source.app_instance_id===p.app_instance_id&&source.mcu_restarts===p.mcu_restarts);
    return {key:active?p.id:null,status:active?{...source,gyro:source.gyro.map((v,i)=>v-p.bias_deg_s[i])}:source};
  }
}

export class ImuPoseTracker {
  constructor(){this.sequence=null;this.sourceUs=null;this.instance=null;this.restarts=null;this.lastNew=null;this.pose=null;this.mode='waiting';this.reason='等待反馈';this.fusion=new GravityKalman();this.gyroReference=new ConfirmedStationaryGyroBias(null);this.gyroReferenceKey=null;}
  setGyroReference(profile){this.gyroReference=new ConfirmedStationaryGyroBias(profile);}
  hold(mode,reason){this.mode=mode;this.reason=reason;if(mode==='paused')this.fusion.reset();return {changed:false,pose:this.pose,mode,reason};}
  consume(message,now,wallNow=Date.now()){
    if(message?.type!=='imu-model-state'||message.version!==1)return null;
    if(!['live','stale','paused'].includes(message.mode))return null;
    if(message.mode!=='live'){
      const allowed=['离线','读取暂停','页面已暂停','反馈过期','反馈未前进','IMU 等待数据','等待反馈'];
      return this.hold(message.mode,allowed.includes(message.reason)?message.reason:'反馈暂停');
    }
    const input=message.status,s=input?{acc:input.acc,gyro:input.gyro,mcu_seq:input.mcu_seq,received:input.received,
      feedback_age_ms:input.feedback_age_ms,imu_ok:input.imu_ok,app_instance_id:input.app_instance_id,mcu_imu_us:input.mcu_imu_us,mcu_restarts:input.mcu_restarts}:null;
    if(!s||s.imu_ok!==true||!Number.isInteger(s.mcu_seq)||s.mcu_seq<0||s.mcu_seq>0xffffffff||
       !Number.isFinite(s.received)||s.received<=0||!Number.isFinite(s.feedback_age_ms)||s.feedback_age_ms<0||s.feedback_age_ms>=150||
       typeof s.app_instance_id!=='string'||!s.app_instance_id||!vector(s.acc)||!vector(s.gyro)||
       !Number.isFinite(Math.hypot(...s.gyro))||!uint32(s.mcu_imu_us)||!uint32(s.mcu_restarts))return this.hold('stale','反馈无效');
    let pose=deriveAttitude(s.acc,this.pose?.roll||0);
    if(!pose)return this.hold('stale','加速度无效');
    if(!Number.isFinite(message.observedAtMs)||message.observedAtMs<=0)return this.hold('stale','采样时间未知');
    const cachedAge=wallNow-message.observedAtMs;
    if(cachedAge+s.feedback_age_ms>=150||cachedAge<0)return this.hold('stale','反馈过期');
    const previous=this.sequence,delta=previous===null?1:(s.mcu_seq-previous)>>>0;
    const sourceStep=this.sourceUs===null?1:(s.mcu_imu_us-this.sourceUs)>>>0;
    const restarted=previous!==null&&(this.instance!==s.app_instance_id||this.restarts!==s.mcu_restarts||delta>=0x80000000||sourceStep>=0x80000000);
    if(restarted){this.sequence=s.mcu_seq;this.sourceUs=s.mcu_imu_us;this.instance=s.app_instance_id;this.restarts=s.mcu_restarts;this.lastNew=null;this.fusion.reset();return {...this.hold('stale','重启后等待新反馈'),sourceEpochChanged:true};}
    if(previous!==null&&(delta===0||sourceStep===0)){if(this.lastNew===null||now-this.lastNew>=150)return this.hold('stale','反馈未前进');return {changed:false,pose:this.pose,mode:this.mode,reason:this.reason};}
    const correction=this.gyroReference.correct(s,wallNow),referenceChanged=this.gyroReferenceKey!==correction.key;
    // A reference transition must not subtract both the old learned bias and
    // the new measured reference, nor integrate between the two gyro bases.
    if(referenceChanged){this.fusion.reset();this.gyroReferenceKey=correction.key;}
    const fused=this.fusion.consume(correction.status);if(fused)pose=fused;
    pose={...pose,gyroBiasCorrected:correction.key!==null,gyroCalibrationChanged:referenceChanged};
    this.sequence=s.mcu_seq;this.sourceUs=s.mcu_imu_us;this.instance=s.app_instance_id;this.restarts=s.mcu_restarts;this.lastNew=now-Math.max(0,cachedAge)-s.feedback_age_ms;this.pose=pose;this.mode='live';this.reason=pose.fused?'卡尔曼融合':'重力姿态';
    return {changed:true,pose,mode:this.mode,reason:this.reason,sourceAgeMs:cachedAge+s.feedback_age_ms};
  }
  tick(now){return this.mode==='live'&&this.lastNew!==null&&now-this.lastNew>=150?this.hold('stale','反馈过期'):null;}
}

/* Display-only adaptive follow and bounded gyro prediction. Raw readings stay unchanged. */
export class ObservedPoseTween {
  constructor(){this.current=null;this.target=null;this.lastAt=null;this.us=null;this.renderAt=null;this.speed=0;this.active=false;}
  static fullQuaternion(pose){return Array.isArray(pose?.quaternion)&&pose.quaternion.length===4&&pose.quaternion.every(Number.isFinite)&&Math.hypot(...pose.quaternion)>1e-8;}
  static multiply(a,b){const [x,y,z,w]=a,[X,Y,Z,W]=b;return [w*X+x*W+y*Z-z*Y,w*Y-x*Z+y*W+z*X,w*Z+x*Y-y*X+z*W,w*W-x*X-y*Y-z*Z];}
  static tilt(pose){
    if(ObservedPoseTween.fullQuaternion(pose)){const n=Math.hypot(...pose.quaternion);return pose.quaternion.map(v=>v/n);}
    const r=pose.roll*Math.PI/180,p=pose.pitch*Math.PI/180;
    const g=vector(pose.gravity)?pose.gravity:[Math.sin(p),-Math.cos(p)*Math.sin(r),-Math.cos(p)*Math.cos(r)];
    // Minimal gravity alignment has no Euler pole spin and makes no yaw claim.
    const q=[-g[1],g[0],0,1-g[2]],length=Math.hypot(...q);
    return length<1e-8?[1,0,0,0]:q.map(v=>v/length);
  }
  static blend(a,b,t){
    let dot=a.reduce((s,v,i)=>s+v*b[i],0),target=b;
    if(dot<0){dot=-dot;target=b.map(v=>-v);}
    dot=Math.min(1,dot);
    const angle=Math.acos(dot),sine=Math.sin(angle);
    const weights=sine<1e-5?[1-t,t]:[Math.sin((1-t)*angle)/sine,Math.sin(t*angle)/sine];
    const q=a.map((v,i)=>v*weights[0]+target[i]*weights[1]),norm=Math.hypot(...q);
    return q.map(v=>v/norm);
  }
  static pose(q,previousRoll=0,previousHeading=null){
    const [x,y,z,w]=q;
    const forward=[1-2*(y*y+z*z),2*(x*y+w*z)],headingObservable=Math.hypot(...forward)>=.01;
    const attitude=deriveAttitude([2*(w*y-x*z),-2*(y*z+w*x),2*(x*x+y*y)-1],previousRoll);
    return {...attitude,roll:unwrapAngle(attitude.roll,previousRoll),quaternion:q,
      headingDeg:headingObservable?unwrapAngle(Math.atan2(forward[1],forward[0])*180/Math.PI,previousHeading):null,headingObservable};
  }
  observe(pose,now,continuous=true,sourceUs=null){
    if(!pose||!Number.isFinite(pose.roll)||!Number.isFinite(pose.pitch)||!Number.isFinite(now))return this.current;
    const q=ObservedPoseTween.tilt(pose),delta=this.us===null||!Number.isInteger(sourceUs)?null:(sourceUs-this.us)>>>0;
    if(!this.current||!continuous||this.lastAt===null||now-this.lastAt>1500||delta!==null&&delta>=0x80000000){
      this.current=ObservedPoseTween.pose(q,pose.roll);this.renderAt=now;this.speed=0;
    }else{
      if(delta===0)return this.current;
      const dot=Math.min(1,Math.abs(q.reduce((sum,v,i)=>sum+v*this.current.quaternion[i],0)));
      const error=2*Math.acos(dot)*180/Math.PI;
      // Sub-degree alternating noise is not a fast movement just because
      // samples are 10ms apart. Large measured changes still catch up promptly.
      this.speed=Math.max(0,error-.6)*30;
    }
    if(vector(pose.gyroRad)&&vector(pose.gravity))this.speed=Math.max(this.speed,
      Math.max(0,(ObservedPoseTween.fullQuaternion(pose)?Math.hypot(...pose.gyroRad):Math.hypot(...GravityKalman.cross(pose.gyroRad,pose.gravity)))*180/Math.PI-1.5));
    this.target={q,gravity:pose.gravity,gyroRad:pose.gyroRad,full:ObservedPoseTween.fullQuaternion(pose)};
    this.us=Number.isInteger(sourceUs)?sourceUs:null;this.lastAt=now;this.active=true;return this.current;
  }
  sample(now){
    if(!this.active||!this.target)return this.current;
    let target=this.target.q;
    // Display-only gyro prediction is capped at 40ms. It never creates data frames.
    if(vector(this.target.gyroRad)&&vector(this.target.gravity)){
      const omega=this.target.gyroRad,speed=Math.hypot(...omega),v=this.target.gravity;
      const predictionWeight=Math.min(1,Math.max(0,(speed*180/Math.PI-3)/12));
      const dt=Math.max(0,Math.min(40,now-this.lastAt))/1000*predictionWeight;
      if(speed>1e-8){
        const axis=omega.map(x=>x/speed),cross=GravityKalman.cross(axis,v),dot=axis.reduce((sum,x,i)=>sum+x*v[i],0);
        const angle=speed*dt,co=Math.cos(angle),si=Math.sin(angle);
        const gravity=v.map((x,i)=>x*co-cross[i]*si+axis[i]*dot*(1-co));
        target=this.target.full?ObservedPoseTween.multiply(this.target.q,[...axis.map(value=>value*Math.sin(angle/2)),Math.cos(angle/2)]):ObservedPoseTween.tilt({roll:this.current.roll,pitch:this.current.pitch,gravity});
      }
    }
    const dt=Math.max(0,Math.min(100,now-this.renderAt));this.renderAt=now;
    const cutoff=2+Math.min(42,this.speed*.65),tau=1000/(2*Math.PI*cutoff);
    const q=ObservedPoseTween.blend(this.current.quaternion,target,1-Math.exp(-dt/tau));
    this.current=ObservedPoseTween.pose(q,this.current.roll,this.current.headingDeg);return this.current;
  }
  freeze(){this.active=false;return this.current;}
}

// Euler roll and yaw are poorly conditioned near vertical pitch, even when the
// 3D quaternion is stable. This quality gate affects presentation only.
export const nearVerticalProjection = pose => Number.isFinite(pose?.pitch)&&Math.abs(pose.pitch)>85;
// Display-only low-confidence Euler projection; never feeds fusion, heading integration, or control.
export function unstableEulerProjection(quaternion,previousRoll=null,previousYaw=null){
  if(!Array.isArray(quaternion)||quaternion.length!==4||!quaternion.every(Number.isFinite))return {roll:null,yaw:null};
  const [x,y,z,w]=quaternion,gravityY=-2*(y*z+w*x),gravityZ=2*(x*x+y*y)-1;
  const forwardX=1-2*(y*y+z*z),forwardY=2*(x*y+w*z),degrees=180/Math.PI;
  const roll=Math.hypot(gravityY,gravityZ)>1e-6?Math.atan2(-gravityY,-gravityZ)*degrees:null;
  const yaw=Math.hypot(forwardX,forwardY)>1e-6?Math.atan2(forwardY,forwardX)*degrees:null;
  return {roll:Number.isFinite(roll)?unwrapAngle(roll,previousRoll):null,
    yaw:Number.isFinite(yaw)?unwrapAngle(yaw,previousYaw):null};
}
export const projectedAngleReadout = (value,observable,unstable,lastStable=null,lowConfidence=null) => {
  const projected=Number.isFinite(value)&&observable!==false?value:
    unstable&&Number.isFinite(lowConfidence)?lowConfidence:null;
  return Number.isFinite(projected)?{value:projected,quality:unstable?'unstable':'current'}:
    Number.isFinite(lastStable)?{value:lastStable,quality:'historical'}:{value:null,quality:'unknown'};
};

/* Real fused observations only. Render prediction never appends a chart point. */
export class PoseAngleHistory {
  constructor(windowMs=10000){this.windowMs=windowMs;this.rows=[];this.us=null;this.sequence=null;this.identity=null;this.segment=null;this.elapsed=0;this.previous=[null,null,null];this.gap=true;}
  markGap(resetHeading=false){this.gap=true;if(resetHeading)this.previous[2]=null;}
  append(pose,source,segment=null){
    if(!pose||!uint32(source?.mcu_imu_us)||!uint32(source?.mcu_seq)||!uint32(source?.mcu_restarts)||typeof source.app_instance_id!=='string')return false;
    const identity=source.app_instance_id+':'+source.mcu_restarts;
    const delta=this.us===null?0:(source.mcu_imu_us-this.us)>>>0;
    const restart=this.identity!==null&&(identity!==this.identity||delta>=0x80000000);
    if(restart){this.rows=[];this.us=null;this.sequence=null;this.elapsed=0;this.previous=[null,null,null];this.gap=true;}
    if(this.us!==null&&delta===0)return false;
    const sequenceStep=this.sequence===null?1:(source.mcu_seq-this.sequence)>>>0;
    if(this.us!==null)this.elapsed+=delta/1000;
    // A latest-only transport can skip sequences without a long observation
    // gap. Tilt endpoints remain measured; heading integration is separately
    // discontinuous because missing angular velocity is not reconstructed.
    const broken=this.gap||delta>100000;
    const headingBroken=broken||sequenceStep!==1||this.segment!==null&&segment!==this.segment;
    const nearVertical=nearVerticalProjection(pose);
    const angles=[nearVertical||pose.rollObservable===false?null:pose.roll,pose.pitch,nearVertical||pose.headingObservable===false?null:pose.headingDeg];
    const values=angles.map((value,i)=>Number.isFinite(value)?unwrapAngle(value,this.previous[i]):null);
    values.forEach((value,i)=>{if(value!==null)this.previous[i]=value;});
    this.rows.push({ms:this.elapsed,values,nearVertical,breakBefore:broken,headingBreakBefore:headingBroken,sourceUs:source.mcu_imu_us,sequence:source.mcu_seq});
    this.rows=this.rows.filter(row=>row.ms>=this.elapsed-this.windowMs).slice(-2000);
    this.us=source.mcu_imu_us;this.sequence=source.mcu_seq;this.identity=identity;this.segment=segment;this.gap=false;
    return true;
  }
}

/* Display curve only: historical quaternion frames, never fabricated MCU samples. */
export class DisplayPoseHistory {
  constructor(windowMs=10000){this.windowMs=windowMs;this.kind='display';this.reset();}
  reset(){this.rows=[];this.elapsed=0;this.identity=null;this.previous=[null,null,null];this.stable=[null,null,null];this.gap=true;}
  markGap(){this.gap=true;}
  append(pose,frame){
    if(frame?.state!=='playing'||!Number.isFinite(frame.sourceTimeMs)||!pose)return false;
    if(this.identity!==null&&this.identity!==frame.sourceEpoch)this.reset();
    const ms=frame.sourceTimeMs;
    if(this.rows.length&&ms<=this.rows.at(-1).ms)return false;
    const nearVertical=nearVerticalProjection(pose);
    const lowConfidence=nearVertical?unstableEulerProjection(pose.quaternion,this.previous[0],this.previous[2]):null;
    const projections=[projectedAngleReadout(pose.roll,pose.rollObservable,nearVertical,this.stable[0],lowConfidence?.roll),
      {value:pose.pitch,quality:Number.isFinite(pose.pitch)?'current':'unknown'},
      projectedAngleReadout(pose.headingDeg,pose.headingObservable,nearVertical,this.stable[2],lowConfidence?.yaw)];
    const values=projections.map((reading,i)=>Number.isFinite(reading.value)?
      reading.quality==='historical'?reading.value:unwrapAngle(reading.value,this.previous[i]):null);
    values.forEach((value,i)=>{
      if(value!==null&&projections[i].quality!=='historical')this.previous[i]=value;
      if(value!==null&&projections[i].quality==='current')this.stable[i]=value;
    });
    this.rows.push({ms,values,quality:projections.map(reading=>reading.quality),nearVertical,
      breakBefore:this.gap,headingBreakBefore:this.gap,historical:true,interpolated:frame.interpolated});
    this.elapsed=ms;this.identity=frame.sourceEpoch;this.gap=false;
    this.rows=this.rows.filter(row=>row.ms>=ms-this.windowMs).slice(-240);
    return true;
  }
}

export function drawPoseAngleChart(canvas,history,ratio=1){
  const ctx=canvas?.getContext?.('2d');if(!ctx)return false;
  const bounds=canvas.getBoundingClientRect(),width=Math.round(bounds.width),height=Math.round(bounds.height);
  if(width<80||height<40)return false;
  const scale=Math.max(1,Math.min(2,ratio||1));
  if(canvas.width!==Math.round(width*scale)||canvas.height!==Math.round(height*scale)){
    canvas.width=Math.round(width*scale);canvas.height=Math.round(height*scale);
  }
  ctx.setTransform(scale,0,0,scale,0,0);ctx.clearRect(0,0,width,height);
  const rows=history.rows,pad={left:48,right:14,top:4,bottom:22},w=width-pad.left-pad.right;
  const lane=(height-pad.top-pad.bottom)/3,start=history.elapsed-history.windowMs;
  const x=time=>pad.left+(time-start)/history.windowMs*w;
  ctx.font='11px "Segoe UI","Microsoft YaHei",sans-serif';ctx.lineWidth=1;
  for(const [axis,color] of ['#68d5e3','#efbd83','#bda8e8'].entries()){
    const top=pad.top+axis*lane+24,bottom=pad.top+(axis+1)*lane-10,h=bottom-top;
    const values=rows.map(row=>row.values[axis]).filter(Number.isFinite);
    // Separate axes avoid flattening pitch under a large unwrapped roll. A
    // minimum 20-degree range avoids magnifying stationary sub-degree noise.
    const min=values.length?Math.min(...values):0,max=values.length?Math.max(...values):0;
    const step=[5,10,20,30,60,90,180,360,720].find(value=>(max-min+4)/4<=value)||Math.ceil((max-min+4)/1440)*360;
    const middle=Math.round((min+max)/2/step)*step;
    const half=Math.ceil(Math.max(10,max-middle+2,middle-min+2)/step)*step;
    const low=middle-half,high=middle+half,y=value=>top+(high-value)/(high-low)*h;
    ctx.fillStyle=color;ctx.textAlign='left';ctx.textBaseline='top';
    ctx.fillText(['横滚','俯仰','相对航向'][axis],pad.left,top-22);
    if(axis!==1&&rows.at(-1)?.nearVertical){ctx.fillStyle='#b1a88d';ctx.textAlign='right';
      ctx.fillText(rows.at(-1)?.quality?.[axis]==='historical'?'近竖直 · 上次值':'近竖直 · 投影不稳定',width-pad.right,top-22);}
    ctx.strokeStyle='#29404d';ctx.lineWidth=1;ctx.fillStyle='#7898a8';
    for(const value of [high,middle,low]){
      const py=y(value);ctx.beginPath();ctx.moveTo(pad.left,py);ctx.lineTo(width-pad.right,py);ctx.stroke();
      ctx.textAlign='right';ctx.textBaseline='middle';ctx.fillText(value.toFixed(0)+'°',pad.left-8,py);
    }
    for(let i=0;i<=4;i++){
      const px=pad.left+w*i/4;ctx.beginPath();ctx.moveTo(px,top);ctx.lineTo(px,bottom);ctx.stroke();
    }
    ctx.save();ctx.beginPath();ctx.rect(pad.left,top,w,h);ctx.clip();
    ctx.strokeStyle=color;ctx.fillStyle=color;ctx.lineWidth=1.8;ctx.lineJoin='round';ctx.lineCap='round';
    ctx.beginPath();let connected=false;
    for(const row of rows){
      const value=row.values[axis];if(!Number.isFinite(value)||row.quality?.[axis]==='historical'){connected=false;continue;}
      if(row.breakBefore||axis===2&&row.headingBreakBefore)connected=false;
      const px=x(row.ms),py=y(value);if(connected)ctx.lineTo(px,py);else ctx.moveTo(px,py);connected=true;
    }
    ctx.stroke();
    // Real isolated observations must remain visible, even with no segment.
    for(let i=0;i<rows.length;i++){
      const row=rows[i],next=rows[i+1],value=row.values[axis];
      if(!Number.isFinite(value))continue;
      if(row.quality?.[axis]==='historical'){
        if(i===rows.length-1){ctx.fillStyle='#b1a88d';ctx.beginPath();ctx.arc(x(row.ms),y(value),2.7,0,Math.PI*2);ctx.fill();ctx.fillStyle=color;}
        continue;
      }
      const isolated=(row.breakBefore||axis===2&&row.headingBreakBefore)&&(!next||next.breakBefore||axis===2&&next.headingBreakBefore);
      if(isolated||i===rows.length-1){ctx.beginPath();ctx.arc(x(row.ms),y(value),i===rows.length-1?2.7:1.3,0,Math.PI*2);ctx.fill();}
    }
    ctx.restore();
  }
  ctx.fillStyle='#91aebb';ctx.textBaseline='bottom';
  for(let i=0;i<=4;i++){
    ctx.textAlign=i===0?'left':i===4?'right':'center';
    ctx.fillText(i===4?(history.kind==='display'?'显示时刻':'最新'):((i/4-1)*history.windowMs/1000).toFixed(1)+'秒',pad.left+w*i/4,height);
  }
  return true;
}

// Display context lifecycle only. Sensor fusion/actuator state is not reset.
export function bindGraphicsLifecycle(canvas,onState){
  let state='ready';
  const change=next=>{if(state===next)return;state=next;onState(next);};
  canvas.addEventListener('webglcontextlost',event=>{event.preventDefault?.();change('lost');});
  canvas.addEventListener('webglcontextrestored',()=>change('waiting'));
  return {get state(){return state;},canRender:()=>state==='ready',
    canDraw:()=>state!=='lost',
    lost:()=>change('lost'),fresh:()=>{if(state==='waiting')change('ready');},report:()=>onState(state)};
}

export const DISPLAY_BUFFER_KEY='duck-display-buffer-ms-v1';
export const displayBufferValue=value=>['200','300','600'].includes(String(value))?Number(value):200;
export function readDisplayBuffer(storage){
  try{return displayBufferValue(storage?.getItem(DISPLAY_BUFFER_KEY));}catch{return 200;}
}
export function changeDisplayBuffer(playback,value){
  const delayMs=displayBufferValue(value);
  if(!playback||playback.delayMs===delayMs)return false;
  const held=playback.held;
  playback.delayMs=delayMs;playback.reset('display-buffer-changed');playback.held=held;
  return true;
}
export function calibrationHighlightIds(message){
  const allowed=[10,11,12,13,14,20,21,22,23,24,30,31,32,33,34];
  if(message?.type!=='imu-model-highlight'||message.version!==1||!Array.isArray(message.ids)||
    message.ids.length>15||new Set(message.ids).size!==message.ids.length||message.ids.some(id=>!allowed.includes(id)))return null;
  return message.ids.slice();
}

async function boot(){
  document.body.dataset.uiRevision='2026-10-17-imu-display-r3';
  const get=id=>document.getElementById(id),tracker=new ImuPoseTracker(),angleHistory=new PoseAngleHistory(),displayHistory=new DisplayPoseHistory();
  let playback=null,shownPose=null;
  // The operator chose a fixed 0.2 s display delay; stored older choices are ignored.
  const displayDelayMs=200;
  let pending=null,pendingNative=null,nativeTracker=null,kinematics=null,modelShift=null,heading=null,headingState=null;
  let model=null,loaded=false,scene,camera,renderer,controls,center,size,THREE,graphics=null;
  let highlightedIds=[],currentTracker=null,pendingCurrent=null,currentRows=[],currentPaintKey='';
  const currentScale=2000;
  let renderCount=0,renderStart=null,lastMetricsAt=0,lastReadingsAt=0;
  let lastStableProjection={roll:null,yaw:null};
  let requestRender=()=>{},stopRender=()=>{},viewportVisible=false;
  function highlightSelection(){
    if(!model)return;
    for(const mesh of model.children){
      const selected=highlightedIds.includes(mesh.userData.motorId);
      const current=currentRows.find(row=>row.id===mesh.userData.motorId),heat=current?.fresh?Math.min(1,Math.abs(current.ma)/currentScale):0;
      mesh.material.color.setHex(mesh.userData.baseColor);
      mesh.material.emissive.setHex(0xff2015);mesh.material.emissiveIntensity=heat*1.35;
      if(mesh.userData.selectionOutline)mesh.userData.selectionOutline.visible=selected;
      if(mesh.userData.holdMarker)mesh.userData.holdMarker.visible=current?.held===true;
    }
    document.body.dataset.highlightedIds=highlightedIds.join(',');
    document.body.dataset.highlightedMeshes=model.children.filter(mesh=>highlightedIds.includes(mesh.userData.motorId)).map(mesh=>mesh.name).join(',');
    requestRender();
  }
  function applyCurrent(rows){
    if(!rows)return;
    const key=rows.map(row=>[row.id,row.ma,row.fresh,row.held].join(':')).join('|');
    if(key===currentPaintKey)return;currentPaintKey=key;currentRows=rows;highlightSelection();
    document.body.dataset.currentFreshIds=rows.filter(row=>row.fresh).map(row=>row.id).join(',');
  }
  let chartDirty=true,lastChartAt=-Infinity;
  const text=(id,value)=>{const element=get(id);if(element&&element.textContent!==value)element.textContent=value;};
  const showBufferChoice=()=>{
    document.body.dataset.playbackDelayMs=String(displayDelayMs);
    const history=get('imu-angle-history');
    if(history)history.title='记录实际显示的历史姿态，目标20FPS；显示缓冲'+(displayDelayMs/1000).toFixed(1)+'秒，仅在已收到的真实端点间插值。真实源缺口超过300ms冻结并断线。';
  };
  showBufferChoice();
  const applyNative=result=>{
    if(!result)return;
    if(result.changed&&model&&kinematics&&graphics?.state!=='lost'){
      try{
        const matrices=kinematics.solve(result.pose.qRadians);
        const ready=model.children.map(mesh=>({mesh,matrix:matrices[mesh.name]}));
        if(ready.some(row=>!row.matrix))throw Error('Missing physical-pose link');
        for(const {mesh,matrix} of ready){mesh.matrix.set(...matrix.flat());mesh.matrix.premultiply(modelShift);mesh.matrixWorldNeedsUpdate=true;}
      }catch{result=nativeTracker.hold('stale','关节模型映射无效');}
      requestRender();
    }
    document.body.dataset.nativeMode=result.mode;
    document.body.dataset.nativeReason=result.reason;
    document.body.dataset.nativeHistorical=String(result.historical);
    if(result.pose){document.body.dataset.nativeSequence=String(result.pose.sequence);document.body.dataset.zeroSha256=result.pose.zeroSha;}
    if(result.pose)document.body.dataset.nativeFrozenIds=result.pose.jointLive.flatMap((live,i)=>live?[]:[result.pose.ids[i]]).join(',');
    const mouthKnown=result.pose?.renderKnown[14]===true;
    const mouthLive=result.pose?.jointLive[14]===true;
    const mouthDegrees=mouthKnown?result.pose.qRadians[14]*180/Math.PI:null;
    document.body.dataset.mouthLive=String(mouthLive);
    document.body.dataset.mouthAngleDeg=mouthDegrees===null?'':mouthDegrees.toFixed(2);
    text('imu-model-mouth',mouthDegrees===null?'—':mouthDegrees.toFixed(1)+'°');
    const mouthReading=get('imu-model-mouth');
    if(mouthReading)mouthReading.title=mouthLive?'ID34新鲜编码器反馈；相对已保存闭嘴零位':mouthKnown?'ID34或头部反馈过期，保留最后嘴部角度':'等待ID34与头部新鲜反馈';
    const scope=document.querySelector('.scope');
    if(scope){
      const known=result.pose?.renderKnown.slice(0,14).some(Boolean);
      const waiting=result.reason==='等待保存机械零位'?'等待机械零位 · q=0参考':'关节反馈待接入 · q=0参考';
      const label=result.mode==='live'?(result.historical?'编码器姿态 · 部分关节冻结':'编码器姿态'):known?'编码器姿态 · 关节反馈已冻结':waiting;
      const mouth=result.pose?.positions[14]!==null&&result.pose?` · 嘴34 ${result.pose.positions[14]}（Δ${result.pose.deltaNative[14]}步；${mouthLive?'新鲜':'历史'}）`:' · 嘴34等待新鲜反馈';
      if(scope.textContent!==label)scope.textContent=label;
      scope.hidden=result.mode==='live'&&!result.historical;
      scope.title='15关节按编码器和当前机械零位重建；嘴巴使用独立下颚及闭嘴零位；安装方向待实物核对，方向翻转仅影响显示。'+mouth+'。IMU估计横滚/俯仰；航向相对启动或归零方向积分，无磁力计会漂移，全局位置未测。';
    }
  };
  const apply=(result,source=null)=>{
    if(!result)return;
    let displayPose=result.pose;
    if(result.sourceEpochChanged||result.changed&&result.pose?.gyroCalibrationChanged){
      playback?.reset(result.sourceEpochChanged?'source-epoch-changed':'gyro-reference-changed');
      displayHistory.reset();shownPose=null;chartDirty=true;document.body.dataset.chartSamples='0';
      lastStableProjection={roll:null,yaw:null};
    }
    playback?.setPaused(result.mode==='paused'||document.hidden||graphics?.state==='lost');
    if(result.changed&&result.mode==='live'&&source&&heading){
      headingState=heading.observe(result.pose,source,performance.now());
      if(headingState?.mode==='live'&&headingState.quaternion)displayPose={...result.pose,quaternion:headingState.quaternion};
      const chartPose=ObservedPoseTween.pose(ObservedPoseTween.tilt(displayPose),displayPose.roll);
      if(angleHistory.append(chartPose,source,headingState?.segment)){
        document.body.dataset.rawSamples=String(angleHistory.rows.length);document.body.dataset.rawSourceUs=String(source.mcu_imu_us);
      }
    }else if(result.mode!=='live'){
      angleHistory.markGap();if(heading)headingState=heading.hold(result.reason);
    }
    document.body.dataset.headingReference=headingState?.reference||'startup';
    document.body.dataset.headingContinuous=String(headingState?.continuous===true);
    document.body.dataset.headingDiscontinuity=headingState?.discontinuity||'';
    if(result.changed&&result.mode==='live')graphics?.fresh();
    if(result.changed&&result.mode==='live'&&source&&playback&&Number.isFinite(result.sourceAgeMs)){
      playback.observe(ObservedPoseTween.tilt(displayPose),{...source,feedback_age_ms:result.sourceAgeMs},performance.now());
    }
    if(graphics&&!graphics.canRender()){
      document.body.dataset.live='false';
      if(get('imu-model-yaw-zero'))get('imu-model-yaw-zero').disabled=true;
      document.body.dataset.mode=graphics.state==='lost'?'graphics-lost':'waiting';
      text('imu-model-state',graphics.state==='lost'?'图形暂停，等待恢复':'画面已恢复 · 保留姿态');
      document.body.dataset.poseHistorical=String(!!shownPose);requestRender();return;
    }
    document.body.dataset.mode=result.mode;
    document.body.dataset.fusion=result.pose?.fused?'gravity-ekf-v1':'acceleration-only';
    document.body.dataset.gyroBias=result.pose?.gyroBiasCorrected?'confirmed-stationary':'none';
    text('imu-receive-state',result.mode==='live'?'当前接收正常':result.mode==='paused'?'当前接收已暂停':'当前接收间断 · 显示使用历史缓冲');
    if(source&&result.changed){
      const recent=angleHistory.rows.filter(row=>row.ms>=angleHistory.elapsed-2000);
      const span=recent.length>1?recent.at(-1).ms-recent[0].ms:0;
      const rate=span>0?(recent.length-1)*1000/span:null;
      text('imu-sample-quality',(rate===null?'—':rate.toFixed(0))+' 采样/秒 · '+result.sourceAgeMs.toFixed(0)+' ms');
      document.body.dataset.sampleRate=rate===null?'':rate.toFixed(1);
    }
    text('imu-history-state','显示姿态曲线 · 历史端点插值');
    if(get('imu-receive-state'))get('imu-receive-state').title=result.reason;
    document.body.dataset.poseHistorical=String(!!shownPose);
    if(get('imu-model-yaw-zero'))get('imu-model-yaw-zero').disabled=result.mode!=='live'||!headingState?.projectionObservable||nearVerticalProjection(result.pose);
    document.body.dataset.live=String(result.mode==='live');
    if(result.mode==='paused'){displayHistory.reset();chartDirty=true;document.body.dataset.chartSamples='0';stopRender();return;}
    requestRender();
  };
  const zeroHeading=()=>{
    const now=performance.now();apply(tracker.tick(now));
    if(document.hidden||tracker.mode!=='live'||!graphics?.canRender()||nearVerticalProjection(tracker.pose))return;
    const zeroed=heading?.zero(now);if(!zeroed)return;
    // A changed reference cannot interpolate through the previous reference.
    headingState=zeroed;playback?.reset('heading-zero');displayHistory.reset();shownPose=null;chartDirty=true;document.body.dataset.chartSamples='0';
    lastStableProjection.yaw=null;
    angleHistory.markGap(true);
    document.body.dataset.headingReference='user-zero';
    document.body.dataset.headingContinuous='true';document.body.dataset.headingDiscontinuity='';lastReadingsAt=-Infinity;
  };
  window.addEventListener('message',event=>{
    if(event.origin!==location.origin||event.source!==window.parent)return;
    if(event.data?.type==='imu-model-highlight'){
      const ids=calibrationHighlightIds(event.data);if(ids!==null){highlightedIds=ids;highlightSelection();}return;
    }
    if(event.data?.type==='motor-current-state'){
      if(!currentTracker){pendingCurrent=event.data;return;}applyCurrent(currentTracker.consume(event.data,performance.now()));return;
    }
    if(event.data?.type==='imu-model-heading-zero'&&event.data.version===1){zeroHeading();return;}
    if(event.data?.type==='native-pose-state'){
      if(document.hidden){pendingNative=null;if(nativeTracker)applyNative(nativeTracker.hold('paused','页面已暂停'));return;}
      if(!loaded){pendingNative=event.data;return;}
      applyNative(nativeTracker.consume(event.data,performance.now()));return;
    }
    if(event.data?.type!=='imu-model-state')return;
    if(document.hidden){pending=null;apply(tracker.hold('paused','页面已暂停'));return;}
    if(!loaded){pending=event.data;return;}
    apply(tracker.consume(event.data,performance.now()),event.data.status);
  });
  document.addEventListener('visibilitychange',()=>{if(document.hidden){pending=null;pendingNative=null;apply(tracker.hold('paused','页面已暂停'));if(nativeTracker)applyNative(nativeTracker.hold('paused','页面已暂停'));}else requestRender();});
  setInterval(()=>{const now=performance.now();apply(tracker.tick(now));if(nativeTracker)applyNative(nativeTracker.tick(now));if(currentTracker)applyCurrent(currentTracker.read(now));},100);
  try{
    const modules=await Promise.all([import('../model-assets/vendor/build/three.module.js'),
      import('../model-assets/vendor/examples/jsm/controls/OrbitControls.js'),import('../model-assets/vendor/examples/jsm/loaders/STLLoader.js'),import('./native-pose.js'),import('./relative-heading.js'),import('./pose-playback.js'),import('./motor-current.js')]);
    THREE=modules[0];const {OrbitControls}=modules[1],{STLLoader}=modules[2];
    nativeTracker=new modules[3].NativePoseTracker();
    currentTracker=new modules[6].MotorCurrentTracker();
    heading=new modules[4].RelativeHeading();
    playback=new modules[5].DelayedPosePlayback({fps:20,delayMs:displayDelayMs,maxGapMs:300});
    try{
      const reference=await Promise.resolve({ok:false});
      if(reference.ok)tracker.setGyroReference(await reference.json());
    }catch{ /* Missing/invalid reference leaves the original fusion behavior. */ }
    const viewport=get('imu-model-viewport');scene=new THREE.Scene();
    scene.add(new THREE.HemisphereLight(0xc9f2ff,0x16212c,2.6));
    const key=new THREE.DirectionalLight(0xdff8ff,3.1);key.position.set(.35,-.45,.45);scene.add(key);
    const rim=new THREE.DirectionalLight(0x46c8e0,2.0);rim.position.set(-.3,.25,.25);scene.add(rim);
    const warm=new THREE.DirectionalLight(0xeab177,.65);warm.position.set(.25,.4,.15);scene.add(warm);
    camera=new THREE.PerspectiveCamera(34,1,.001,10);camera.up.set(0,0,1);
    renderer=new THREE.WebGLRenderer({antialias:true,alpha:true});renderer.setPixelRatio(Math.min(devicePixelRatio,1.5));renderer.setClearColor(0x071621,0);renderer.toneMapping=THREE.ACESFilmicToneMapping;renderer.toneMappingExposure=1.05;viewport.append(renderer.domElement);
    graphics=bindGraphicsLifecycle(renderer.domElement,state=>{
      document.body.dataset.graphics=state;document.body.dataset.loaded=String(loaded&&state==='ready');
      if(state!=='ready'){
        renderCount=0;renderStart=null;document.body.dataset.renderFps='0';
        playback.setPaused(state==='lost');if(state==='lost')stopRender();
        apply(tracker.hold('stale',state==='lost'?'图形上下文失效':'等待新鲜数据'));
        if(nativeTracker)applyNative(nativeTracker.hold('stale','等待新鲜关节反馈'));
      }
      if(window.parent!==window)window.parent.postMessage({type:'imu-model-graphics-state',version:1,state,loaded:loaded&&state==='ready'},location.origin);
      requestRender();
    });
    controls=new OrbitControls(camera,renderer.domElement);controls.enableDamping=true;controls.enablePan=false;controls.minDistance=.20;controls.maxDistance=.9;
    const response=await fetch('../model-assets/model-data-mouth.json');if(!response.ok)throw Error('模型清单未读取');
    const data=await response.json(),pose=data.poses?.['q-zero'];
    if(data.records?.length!==16||!pose||!pose.records.trunk_base)throw Error('模型基线不完整');
    kinematics=modules[3].buildNativeKinematics(data);
    model=new THREE.Group();scene.add(model);
    const pivot=new THREE.Vector3(...pose.records.trunk_base.slice(0,3).map(row=>row[3]));
    const shift=new THREE.Matrix4().makeTranslation(-pivot.x,-pivot.y,-pivot.z),loader=new STLLoader();
    modelShift=shift;
    await Promise.all(data.records.map(async record=>{
      const meshFile=record.name==='head_roll'?data.mouth_visualization.head_mesh:record.name+'.stl';
      const geometry=await loader.loadAsync('../model-assets/'+meshFile);geometry.computeVertexNormals();
      const color=record.name==='mouth_jaw'?0xd4a454:record.name.includes('ankle')?0xb69467:record.name.includes('head')?0xa1bac8:record.name==='trunk_base'?0x425d6b:0x698593;
      const material=new THREE.MeshStandardMaterial({color,roughness:.40,metalness:.32});
      const mesh=new THREE.Mesh(geometry,material);mesh.name=record.name;mesh.matrixAutoUpdate=false;
      mesh.userData.motorId=Number(Object.keys(modules[3].LINK_BY_ID).find(id=>modules[3].LINK_BY_ID[id]===record.name));mesh.userData.baseColor=color;
      geometry.computeBoundingSphere();
      if(Number.isInteger(mesh.userData.motorId)){
        const outline=new THREE.LineSegments(new THREE.EdgesGeometry(geometry,30),new THREE.LineBasicMaterial({color:0x54c9e8,transparent:true,opacity:.9,depthTest:false}));
        outline.visible=false;outline.renderOrder=2;mesh.add(outline);mesh.userData.selectionOutline=outline;
        const marker=new THREE.Mesh(new THREE.SphereGeometry(.0022,10,6),new THREE.MeshBasicMaterial({color:0x72d9aa,depthTest:false}));
        marker.position.copy(geometry.boundingSphere.center);marker.visible=false;marker.renderOrder=3;mesh.add(marker);mesh.userData.holdMarker=marker;
      }
      mesh.matrix.set(...pose.records[record.name].flat());mesh.matrix.premultiply(shift);model.add(mesh);
    }));
    scene.updateMatrixWorld(true);const box=new THREE.Box3().setFromObject(model);center=box.getCenter(new THREE.Vector3());size=box.getSize(new THREE.Vector3());
    const grid=new THREE.GridHelper(.38,12,0x3b7185,0x183647);grid.rotation.x=Math.PI/2;grid.position.z=box.min.z-.018;grid.material.transparent=true;grid.material.opacity=.26;scene.add(grid);
    const circleGeometry=new THREE.BufferGeometry().setFromPoints(Array.from({length:81},(_,i)=>new THREE.Vector3(Math.cos(i*Math.PI/40)*.108,Math.sin(i*Math.PI/40)*.108,grid.position.z)));
    scene.add(new THREE.Line(circleGeometry,new THREE.LineBasicMaterial({color:0x428399,transparent:true,opacity:.38})));
    function home(){const span=Math.max(size.x,size.y,size.z);camera.position.copy(center).add(new THREE.Vector3(span*1.15,-span*1.65,span*.95));controls.target.copy(center);controls.update();}
    let canvasWidth=0,canvasHeight=0,canvasRatio=0;
    function resize(){
      const bounds=viewport.getBoundingClientRect(),width=Math.round(bounds.width),height=Math.round(bounds.height),ratio=renderer.getPixelRatio();
      viewportVisible=width>=2&&height>=2;
      if(!viewportVisible||width===canvasWidth&&height===canvasHeight&&ratio===canvasRatio)return;
      canvasWidth=width;canvasHeight=height;canvasRatio=ratio;
      camera.aspect=width/height;camera.updateProjectionMatrix();renderer.setSize(width,height,false);requestRender();
    }
    get('imu-model-camera').addEventListener('click',home);new ResizeObserver(resize).observe(viewport);home();resize();
    if(get('imu-angle-chart'))new ResizeObserver(()=>{chartDirty=true;requestRender();}).observe(get('imu-angle-chart'));
    document.body.dataset.loaded='true';document.body.dataset.meshCount=String(model.children.length);document.body.dataset.modelVersion=data.version;document.body.dataset.referencePose='q-zero';
    loaded=true;
    if(pendingCurrent){applyCurrent(currentTracker.consume(pendingCurrent,performance.now()));pendingCurrent=null;}
    highlightSelection();
    const raycaster=new THREE.Raycaster(),pointer=new THREE.Vector2();let pointerStart=null;
    renderer.domElement.addEventListener('pointerdown',event=>{pointerStart={x:event.clientX,y:event.clientY};});
    renderer.domElement.addEventListener('pointerup',event=>{
      const start=pointerStart;pointerStart=null;
      if(!start||Math.hypot(event.clientX-start.x,event.clientY-start.y)>6)return;
      const bounds=renderer.domElement.getBoundingClientRect();
      pointer.set((event.clientX-bounds.left)/bounds.width*2-1,-(event.clientY-bounds.top)/bounds.height*2+1);
      raycaster.setFromCamera(pointer,camera);
      const hit=raycaster.intersectObjects(model.children,false).find(item=>Number.isInteger(item.object.userData.motorId));
      if(hit&&window.parent!==window)window.parent.postMessage({type:'imu-model-joint-selected',version:1,id:hit.object.userData.motorId},location.origin);
    });
    graphics.report();
    if(document.hidden){pending=null;pendingNative=null;apply(tracker.hold('paused','页面已暂停'));applyNative(nativeTracker.hold('paused','页面已暂停'));}
    else{apply(tracker.hold('waiting','等待反馈'));applyNative(nativeTracker.hold('waiting','等待关节反馈'));
      if(pending){apply(tracker.consume(pending,performance.now()),pending.status);pending=null;}
      if(pendingNative){applyNative(nativeTracker.consume(pendingNative,performance.now()));pendingNative=null;}}
    let renderRequest=null,drawRequested=false,inRender=false,nextDrawAt=0;
    stopRender=()=>{if(renderRequest!==null)clearTimeout(renderRequest);renderRequest=null;};
    function render(){
      renderRequest=null;const now=performance.now();
      if(now+.01<nextDrawAt){requestRender();return;}
      let continuePlayback=false,cameraChanged=false;
      inRender=true;
      try{
        apply(tracker.tick(now));
        if(document.hidden||!graphics.canDraw()||tracker.mode==='paused'||!viewportVisible){renderCount=0;renderStart=null;return;}
        const frame=playback.step(now),draw=drawRequested||!!frame;drawRequested=false;
        if(frame){
          document.body.dataset.playbackState=frame.state;
          document.body.dataset.playbackClock=frame.scheduleClock||'waiting';
          document.body.dataset.playbackBufferAheadMs=Number.isFinite(frame.bufferAheadMs)?frame.bufferAheadMs.toFixed(1):'';
          document.body.dataset.playbackDelayMs=String(frame.nominalDelayMs);
          document.body.dataset.playbackSourceMs=frame.sourceTimeMs===null?'':frame.sourceTimeMs.toFixed(1);
          document.body.dataset.playbackKnownAgeMs=frame.knownEndpointAgesMs?Math.max(...frame.knownEndpointAgesMs).toFixed(1):'';
          const bufferSeconds=(frame.nominalDelayMs/1000).toFixed(1);
          text('imu-model-state',frame.state==='playing'?'平滑显示20FPS目标 · 显示缓冲'+bufferSeconds+'秒':frame.state==='buffering'?'准备显示 · 填充'+bufferSeconds+'秒缓冲':'等待数据 · 已保留最后画面');
          get('imu-model-state').title=frame.state==='gap'?'真实源数据缺口超过300ms，未补造这段运动':frame.state==='underflow'?'历史缓冲已用完，等待新的真实端点':'按已收到端点插值显示；缓冲是刻意增加的延迟，不是原始采样或总网络延迟';
          if(frame.quaternion){
            shownPose=ObservedPoseTween.pose(frame.quaternion,shownPose?.roll??tracker.pose?.roll??0,shownPose?.headingDeg??null);
            model.quaternion.fromArray(shownPose.quaternion);
            document.body.dataset.poseHistorical='true';
            if(displayHistory.append(shownPose,frame)){
              chartDirty=true;document.body.dataset.chartSamples=String(displayHistory.rows.length);
              document.body.dataset.chartKind='display-history-interpolation';
            }
            if(frame.state!=='playing')displayHistory.markGap();
          }
          continuePlayback=playback.rows.length>0&&frame.state!=='underflow';
        }
        if(shownPose&&now-lastReadingsAt>=100){
          const visual=shownPose,nearVertical=nearVerticalProjection(visual);
          const lowConfidence=nearVertical?unstableEulerProjection(visual.quaternion,lastStableProjection.roll,lastStableProjection.yaw):null;
          const roll=projectedAngleReadout(visual.roll,visual.rollObservable,nearVertical,lastStableProjection.roll,lowConfidence?.roll);
          const yaw=projectedAngleReadout(visual.headingDeg,visual.headingObservable,nearVertical,lastStableProjection.yaw,lowConfidence?.yaw);
          if(roll.quality==='current')lastStableProjection.roll=roll.value;
          if(yaw.quality==='current')lastStableProjection.yaw=yaw.value;
          const label=reading=>reading.value===null?'—':
            (reading.quality==='unstable'?'≈':reading.quality==='historical'?'上次 ':'')+reading.value.toFixed(1)+'°';
          text('imu-model-pitch',visual.pitch.toFixed(1)+'°');text('imu-model-roll',label(roll));
          text('imu-model-yaw',label(yaw));
          text('imu-projection-note',(tracker.pose?.gyroBiasCorrected?'本会话临时零偏补偿':'未做静置零偏补偿')+
            (nearVertical?' · 近竖直：≈为不稳定投影，“上次”为历史值':''));
          get('imu-model-roll').dataset.quality=roll.quality;get('imu-model-yaw').dataset.quality=yaw.quality;
          get('imu-model-roll').title=roll.quality==='historical'?'接近竖直，显示上次可靠投影，不代表当前角度':
            roll.quality==='unstable'?'接近竖直，当前投影对小扰动敏感':'横滚角度来自历史显示姿态';
          get('imu-model-yaw').title=yaw.quality==='historical'?'接近竖直，显示上次可靠相对航向，不代表当前角度':
            yaw.quality==='unstable'?'接近竖直，相对航向投影不稳定且陀螺积分会漂移':'历史显示姿态的相对航向；陀螺积分仍会漂移';
          document.body.dataset.headingMode=yaw.quality==='current'?'historical-relative':yaw.quality==='unstable'?'unstable-relative':
            yaw.quality==='historical'?'last-observed-relative':'unavailable';
          document.body.dataset.yaw=yaw.quality==='current'||yaw.quality==='unstable'?yaw.value.toFixed(3):'';
          get('imu-model-yaw-zero').disabled=tracker.mode!=='live'||!headingState?.projectionObservable||nearVerticalProjection(tracker.pose);
          get('imu-model-yaw-zero').title='仅将当前新鲜观测朝向设为显示零度，并重新填充显示缓冲';
          document.body.dataset.roll=roll.quality==='current'||roll.quality==='unstable'?roll.value.toFixed(3):'';
          document.body.dataset.pitch=visual.pitch.toFixed(3);lastReadingsAt=now;
        }
        cameraChanged=controls.update();if(!draw&&!cameraChanged)return;
        if(chartDirty&&now-lastChartAt>=50){
          if(drawPoseAngleChart(get('imu-angle-chart'),displayHistory,devicePixelRatio)){chartDirty=false;lastChartAt=now;}
        }
        renderer.render(scene,camera);nextDrawAt=now+50;
        if(renderer.getContext().isContextLost()){graphics.lost();return;}
        renderCount++;
        if(renderStart===null)renderStart=now;
        if(now-renderStart>=1000){
          const fps=(renderCount-1)*1000/(now-renderStart);document.body.dataset.renderFps=fps.toFixed(1);
          if(window.parent!==window&&now-lastMetricsAt>=900){window.parent.postMessage({type:'imu-model-render-metrics',version:1,fps,measured:true},location.origin);lastMetricsAt=now;}
          if(fps<16&&renderer.getPixelRatio()>1){renderer.setPixelRatio(1);resize();}
          renderCount=1;renderStart=now;
        }
      }finally{inRender=false;}
      if(continuePlayback||cameraChanged||drawRequested)requestRender();
    }
    requestRender=()=>{
      drawRequested=true;
      if(!inRender&&renderRequest===null&&!document.hidden&&graphics.canDraw()&&tracker.mode!=='paused'&&viewportVisible){
        const now=performance.now(),deadline=Math.max(nextDrawAt,playback.nextFrameAtMs??now);
        renderRequest=setTimeout(render,Math.max(0,deadline-now));
      }
    };
    controls.addEventListener('change',requestRender);requestRender();
    get('imu-model-yaw-zero').addEventListener('click',event=>{if(event.isTrusted===true)zeroHeading();});
    if(window.parent!==window)window.parent.postMessage({type:'imu-model-ready',version:1,meshes:model.children.length,modelVersion:data.version,nativePoseVersion:1},location.origin);
  }catch(error){document.body.dataset.loaded='false';apply(tracker.hold('stale','模型未加载'));get('imu-model-state').title=String(error.message);}
}
if(typeof document!=='undefined'&&typeof window!=='undefined')boot();
