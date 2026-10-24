import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import vm from 'node:vm';
import {OfficialPosePlayback} from '../pc/assets/official-pose-playback.js';

const state=(seq,uptime,arrival)=>({mcu_seq:seq,uptime_s:uptime,pc_observed_monotonic_s:arrival,
  pc_host_epoch:1,feedback_age_ms:2,full_robot_ready:true,angles:Array(15).fill(seq)});
test('zero-buffer display shows the newest burst endpoint immediately and never replays backlog',()=>{
  const player=new OfficialPosePlayback({fps:40,delayMs:0});
  const poseOf=()=>({quaternion:[0,0,0,1]});
  player.observeBatch(state(1,10,40),[],poseOf,1000);
  assert.equal(player.step(1000).jointAnglesDegrees[0],1);
  player.observeBatch(state(8,10.35,40.6),[state(2,10.05,40.5),state(3,10.1,40.51)],poseOf,1600);
  const frame=player.step(1600);
  assert.equal(frame.jointAnglesDegrees[0],8);
  assert.equal(frame.nominalDelayMs,0);
  assert.equal(frame.sourceGap,true);
  assert.equal(frame.interpolated,false);
  assert.equal(player.rows.length,1);
  assert.equal(frame.knownEndpointAgesMs[0],2);
  assert.equal(player.step(1800).state,'underflow');
  player.observeBatch(state(9,10.4,40.65),[],poseOf,1810);
  assert.equal(player.step(1825).jointAnglesDegrees[0],9,'recovery never waits for an old schedule');
  assert.equal(player.observe([0,0,0,1],state(8,10.35,40.7),1850),false);
});
test('batched Wi-Fi receipt does not compress the real pose interval',()=>{
  const player=new OfficialPosePlayback({fps:40,delayMs:350,maxGapMs:300});
  player.observe([0,0,0,1],state(1,10,40),1000);
  player.observe([0,0,0,1],state(2,10.05,40.2),1200);
  player.observe([0,0,0,1],state(3,10.1,40.201),1201);
  assert.equal(player.rows[2].sourceMs-player.rows[1].sourceMs,50);
  const frame=player.step(1425);
  assert.equal(frame.state,'playing');
  assert.equal(frame.scheduleClock,'robot-uptime');
  assert.equal(frame.jointAnglesDegrees[0],2.5);
  assert.deepEqual(frame.endpointObservedMs,[40200,40201]);
});
test('a real source gap remains a gap despite quick packet delivery',()=>{
  const player=new OfficialPosePlayback({fps:40,delayMs:350,maxGapMs:300});
  player.observe([0,0,0,1],state(1,10,40),1000);
  player.observe([0,0,0,1],state(2,10.5,40.501),1501);
  assert.equal(player.step(1600).state,'gap');
});
test('source clock regression or stale sensor data never advances playback',()=>{
  const player=new OfficialPosePlayback({fps:40,delayMs:350});
  assert.equal(player.observe([0,0,0,1],state(1,10,40),1000),true);
  assert.equal(player.observe([0,0,0,1],state(2,9.99,40.05),1050),false);
  assert.equal(player.observe([0,0,0,1],{...state(3,10.1,40.1),feedback_age_ms:151},1100),false);
  assert.equal(player.rows.length,1);
});
test('current delivery staleness retains real source endpoints and their joint angles',()=>{
  const player=new OfficialPosePlayback({fps:40,delayMs:350});
  const raw={...state(1,10,40),pc_delivery_age_lower_bound_ms:300};
  const aged={...state(2,10.05,40.001),feedback_age_ms:302,native_feedback_age_ms:2,
    pc_delivery_age_lower_bound_ms:300,full_robot_ready:false,native_full_robot_ready:true};
  assert.equal(player.observe([0,0,0,1],raw,1000),true);
  assert.equal(player.observe([0,0,0,1],aged,1001),true);
  const frame=player.step(1375);
  assert.equal(frame.state,'playing');
  assert.equal(frame.jointAnglesDegrees[0],1.5);
  assert.equal(frame.currentTelemetry,false);
  assert.deepEqual(frame.knownEndpointAgesMs,[677,676]);
  assert.equal(player.observe([0,0,0,1],{...state(3,10.1,40.002),
    feedback_age_ms:400,native_feedback_age_ms:150},1002),false);
});
test('a changed robot epoch clears old source timing and retired epochs stay rejected',()=>{
  const player=new OfficialPosePlayback({fps:40,delayMs:350});
  player.observe([0,0,0,1],state(1,10,40),1000);
  assert.equal(player.observe([0,0,0,1],{...state(1,.1,41),pc_host_epoch:2},2000),true);
  assert.equal(player.rows.length,1);
  assert.equal(player.observe([0,0,0,1],state(2,10.1,41.1),2100),false);
});
test('the picture warning wins over fresh receipt while a real source gap holds the old pose',()=>{
  const source=fs.readFileSync(new URL('../pc/assets/imu-model.js',import.meta.url),'utf8');
  const paint=source.slice(source.indexOf('  const paintOfficialReception='),source.indexOf('  const showBufferChoice='));
  for(const state of ['gap','underflow']){
    const labels={},context={replayFrame:null,tracker:{mode:'live'},document:{body:{dataset:{playbackState:state}}},
      text:(key,value)=>labels[key]=value,loaded:true,hasReceivedLive:true,officialGapSince:null,
      receiveRecoveryMs:5000,lastReceiveGapAt:-Infinity,shownPose:{}};
    vm.runInNewContext(paint+'\npaintOfficialReception(1600);',context);
    assert.match(labels['imu-model-state'],/保留姿态/);
    assert.equal(context.document.body.dataset.mode,state);
    assert.equal(context.document.body.dataset.live,'true','receipt freshness is still separate');
  }
});
