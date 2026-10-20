#pragma once
#include <stdint.h>
#include <math.h>

// Display only. Quaternion/gravity equations are ported from the existing
// model-runtime/host/imu_attitude.py and assets/relative-heading.js.
// Reference is the complete first valid attitude, not an Euler forward heading.
namespace DisplayMath {
constexpr float RADIANS_PI=3.14159265358979323846f;
struct V {float x,y,z;};
struct Q {float x,y,z,w;};
inline V cross(V a,V b){return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};}
inline float dot(V a,V b){return a.x*b.x+a.y*b.y+a.z*b.z;}
inline float length(V v){return sqrtf(dot(v,v));}
inline V unit(V v){const float n=length(v);return n>1e-8f?V{v.x/n,v.y/n,v.z/n}:V{0,0,-1};}
inline Q unit(Q q){const float n=sqrtf(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w);return n>1e-8f?Q{q.x/n,q.y/n,q.z/n,q.w/n}:Q{0,0,0,1};}
inline Q multiply(Q a,Q b){return {a.w*b.x+a.x*b.w+a.y*b.z-a.z*b.y,a.w*b.y-a.x*b.z+a.y*b.w+a.z*b.x,a.w*b.z+a.x*b.y-a.y*b.x+a.z*b.w,a.w*b.w-a.x*b.x-a.y*b.y-a.z*b.z};}
inline Q conjugate(Q q){return {-q.x,-q.y,-q.z,q.w};}
inline V rotate(Q q,V v){const V t=cross({q.x,q.y,q.z},v),u=cross({q.x,q.y,q.z},t);return {v.x+2*(q.w*t.x+u.x),v.y+2*(q.w*t.y+u.y),v.z+2*(q.w*t.z+u.z)};}
inline Q tilt(V g){return g.z>.999999f?Q{1,0,0,0}:unit(Q{-g.y,g.x,0,1-g.z});}
inline Q integrate(Q q,V omega,float dt){
  const float speed=length(omega);if(speed<1e-8f)return q;
  const float half=speed*dt*.5f,f=sinf(half)/speed;
  return unit(multiply(q,{omega.x*f,omega.y*f,omega.z*f,cosf(half)}));
}
inline Q align(Q q,V g){
  const V measured=unit(rotate(q,g)),down={0,0,-1},axis=cross(measured,down);
  const float co=fmaxf(-1,fminf(1,dot(measured,down)));
  Q correction=co<-.999999f?Q{1,0,0,0}:unit(Q{axis.x,axis.y,axis.z,1+co});
  return unit(multiply(correction,q));
}
}

// One explicitly selected power-on stationary fit. Raw/control IMU is untouched.
struct BootGyroBias {
  enum Phase {CALIBRATING,READY,FAILED};
  Phase phase=CALIBRATING;
  uint32_t samples=0,firstUs=0,lastUs=0;
  DisplayMath::V mean={0,0,0},m2={0,0,0},firstGravity={0,0,-1};
  bool observe(DisplayMath::V acc,DisplayMath::V angular,uint32_t us,bool ok){
    using namespace DisplayMath;
    if(phase!=CALIBRATING)return false;
    const float norm=length(acc),maxBias=20*RADIANS_PI/180.f;
    if(!ok||!isfinite(norm)||fabsf(norm/9.8f-1)>.1f||!isfinite(length(angular))||
       fabsf(angular.x)>maxBias||fabsf(angular.y)>maxBias||fabsf(angular.z)>maxBias){phase=FAILED;return false;}
    const V g=unit(acc);
    if(samples){
      const uint32_t step=us-lastUs;
      if(!step)return false;
      if(step>=0x80000000u||dot(g,firstGravity)<.999390827f){phase=FAILED;return false;}
    }else{firstUs=us;firstGravity=g;}
    lastUs=us;++samples;
    const V delta={angular.x-mean.x,angular.y-mean.y,angular.z-mean.z};
    mean={mean.x+delta.x/samples,mean.y+delta.y/samples,mean.z+delta.z/samples};
    m2={m2.x+delta.x*(angular.x-mean.x),m2.y+delta.y*(angular.y-mean.y),m2.z+delta.z*(angular.z-mean.z)};
    if(samples>=200&&us-firstUs>=2000000u){
      const float limit=.35f*RADIANS_PI/180.f;
      const float maximum=limit*limit*(samples-1);
      phase=m2.x<=maximum&&m2.y<=maximum&&m2.z<=maximum?READY:FAILED;
    }
    return true;
  }
};

class DisplayAttitude {
 public:
  BootGyroBias bias;
  bool valid=false,continuous=true,haveHeading=false,heldHeading=false;
  bool missing=false;
  uint32_t sourceUs=0,segment=0;
  DisplayMath::Q q={0,0,0,1},powerOn={0,0,0,1},relative={0,0,0,1};
  DisplayMath::V gravity={0,0,-1},gyro={0,0,0};
  float headingDeg=0;
  // Swing/twist about world gravity Z. It is well-defined at +/-90deg pitch;
  // only a relative 180deg horizontal swing truly loses its twist component.
  bool updateTwist(){
    relative=DisplayMath::unit(DisplayMath::multiply(q,DisplayMath::conjugate(powerOn)));
    if(hypotf(relative.z,relative.w)<.01f){if(haveHeading&&!heldHeading)++segment;continuous=false;heldHeading=haveHeading;return false;}
    if(heldHeading){++segment;continuous=false;}
    const float raw=2*atan2f(relative.z,relative.w)*180/DisplayMath::RADIANS_PI;
    headingDeg=haveHeading?headingDeg+remainderf(raw-headingDeg,360.f):raw;
    haveHeading=true;heldHeading=false;return true;
  }
  bool observe(DisplayMath::V acc,DisplayMath::V angularRadians,uint32_t us,bool ok=true){
    using namespace DisplayMath;
    if(bias.phase!=BootGyroBias::READY){
      const bool sampled=bias.observe(acc,angularRadians,us,ok);
      if(sampled)sourceUs=us;
      if(bias.phase!=BootGyroBias::READY)return false;
    }
    angularRadians={angularRadians.x-bias.mean.x,angularRadians.y-bias.mean.y,angularRadians.z-bias.mean.z};
    const float norm=length(acc);
    if(!ok||!isfinite(norm)||norm<.1f||!isfinite(length(angularRadians))){missing=true;return false;}
    const V measured=unit(acc);
    if(!valid){
      gravity=measured;gyro=angularRadians;q=tilt(gravity);powerOn=q;
      valid=true;sourceUs=us;segment=1;missing=false;updateTwist();return true;
    }
    const uint32_t step=us-sourceUs;
    if(!step)return false;
    continuous=true;
    if(missing||step>100000u){
      // Missing angular velocity is never integrated or used to reset power-on.
      gravity=measured;gyro=angularRadians;q=align(q,gravity);
      continuous=false;++segment;missing=false;
    }else{
      const float dt=step/1000000.f;
      const V oldGyro=gyro;
      V predicted={gravity.x+(angularRadians.z*gravity.y-angularRadians.y*gravity.z)*dt,
                   gravity.y+(angularRadians.x*gravity.z-angularRadians.z*gravity.x)*dt,
                   gravity.z+(angularRadians.y*gravity.x-angularRadians.x*gravity.y)*dt};
      predicted=unit(predicted);
      const float trust=fmaxf(0,1-fabsf(norm/9.8f-1)/.2f),gain=(1-expf(-dt/.3f))*trust;
      gravity=unit(V{(1-gain)*predicted.x+gain*measured.x,(1-gain)*predicted.y+gain*measured.y,(1-gain)*predicted.z+gain*measured.z});
      const float ga=expf(-.69314718056f*dt/.01f);
      gyro={ga*oldGyro.x+(1-ga)*angularRadians.x,ga*oldGyro.y+(1-ga)*angularRadians.y,ga*oldGyro.z+(1-ga)*angularRadians.z};
      const V omega={(oldGyro.x+gyro.x)*.5f,(oldGyro.y+gyro.y)*.5f,(oldGyro.z+gyro.z)*.5f};
      q=align(integrate(q,omega,dt),gravity);
    }
    sourceUs=us;updateTwist();return true;
  }
};

// Pure scheduler: one read-only READ62 of ID10 per >=2s, only after complete
// servo feedback and with >=1ms left in the existing receive window. No waits.
struct VoltageSampler {
  bool attempted=false,pending=false,valid=false;
  uint8_t raw=0;
  uint32_t lastAttemptUs=0,sampleUs=0,attempts=0,replies=0,skipped=0;
  bool request(uint32_t now,uint16_t mask,uint32_t remainingUs,bool busIdle){
    if(pending||(!busIdle)||(attempted&&now-lastAttemptUs<2000000u))return false;
    if(mask!=0x7fff||remainingUs<1000u){++skipped;return false;}
    attempted=pending=true;lastAttemptUs=now;++attempts;return true;
  }
  void reply(uint8_t id,uint8_t value,uint8_t error,uint32_t now){
    if(!pending||id!=10||error||now-lastAttemptUs>6000u)return;
    raw=value;valid=true;sampleUs=now;pending=false;++replies;
  }
  void endWindow(){pending=false;}
};

struct __attribute__((packed)) DisplayPacket {
  char magic[4];
  uint8_t version,flags,voltageId,voltageRaw;
  uint32_t bootLow,bootHigh,seq,sampleUs,imuUs,segment;
  float quaternion[4],relative[4],gravity[3],headingDeg;
  uint32_t voltageSampleUs,voltageAttempts,voltageReplies,voltageSkipped;
  uint32_t computeUs,maxComputeUs,voltageRequestUs,maxVoltageRequestUs;
  float biasGyroDegrees[3];
  uint32_t biasSamples;
};
static_assert(sizeof(DisplayPacket)==128,"Display telemetry v2 ABI");
