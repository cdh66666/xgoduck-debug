import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
const source=fs.readFileSync(new URL('../pc/assets/imu-model.js',import.meta.url),'utf8');
const {officialWorkbenchPose,ObservedPoseTween}=await import('data:text/javascript;base64,'+Buffer.from(source).toString('base64'));
const status=(extra={})=>({imu_ok:true,gravity:[0,0,-1],mcu_seq:9,feedback_age_ms:3,pc_observed_monotonic_s:40,pc_host_epoch:1,...extra});
test('official gravity uses renderer convention without creating heading or a sensor clock',()=>{
  const p=officialWorkbenchPose(status());
  assert.ok(p.quaternion.every((v,i)=>Math.abs(v-[0,0,0,1][i])<1e-10));assert.equal(p.headingObservable,false);
  assert.equal(p.headingDeg,null);assert.equal(p.boardHeadingAvailable,false);
  assert.equal(p.mcu_imu_us,undefined);assert.equal(p.gyroBiasCorrected,false);
});
test('tipped official gravity produces the measured pitch without gyro integration',()=>{
  const p=officialWorkbenchPose(status({gravity:[Math.SQRT1_2,0,-Math.SQRT1_2],gyro:[0,-7.5,0]}));
  assert.ok(Math.abs(p.pitch-45)<1e-8);
  const reconstructed=ObservedPoseTween.pose(p.quaternion);
  assert.ok(Math.abs(reconstructed.pitch-45)<1e-8);
});
test('missing, malformed and stale gravity never becomes a fresh pose',()=>{
  for(const change of [{gravity:null},{gravity:[0,0,0]},{gravity:[NaN,0,-1]},{feedback_age_ms:150},{imu_ok:false},{pc_observed_monotonic_s:undefined}])assert.equal(officialWorkbenchPose(status(change)),null);
});
