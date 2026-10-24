import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import vm from 'node:vm';
const source=fs.readFileSync(new URL('../pc/assets/official-workbench.js',import.meta.url),'utf8');
const handler=source.slice(source.indexOf('  async function handleFeedbackGap('),source.indexOf('  async function poll(){'));
function harness(){
 const c={owned:true,controlledMode:'policy',sessionPending:null,feedbackGapAt:null,lastCommandAckAt:1000,now:1000,stops:0,releases:0,messages:[],performance:{now:()=>c.now},keyboard:{releaseAll:()=>c.releases++},note:m=>c.messages.push(m),loseOwnership:()=>{c.owned=false;c.lastCommandAckAt=null;},stop:async()=>{c.stops++;c.owned=false;}};
 vm.createContext(c);vm.runInContext(handler,c);return c;
}
test('one-second telemetry-only gap sends zero intent without requesting OFF while control ACKs continue',async()=>{
 const c=harness();
 for(const now of [1000,1250,1500,1750,2000,2250]){c.now=now;c.lastCommandAckAt=now-50;assert.equal(await c.handleFeedbackGap(false),false);}
 assert.equal(c.stops,0);assert.equal(c.releases,6);assert.equal(c.feedbackGapAt,1000);
 const packetDeclaration=source.split('\n').find(line=>line.trim().startsWith('const packet='));
 for(const change of [{sessionPending:{mode:'policy'}},{feedbackGapAt:1000},{fresh:false}]){
  const context={controlledMode:'policy',sessionPending:null,feedbackGapAt:null,fresh:true,...change,keyboard:{packet(){throw Error('nonzero keyboard path reached');}}};
  context.homeFeedbackFresh=()=>context.fresh;
  const packet=vm.runInNewContext(packetDeclaration+'\npacket',context);
  assert.deepEqual(Array.from(packet.twist),[0,0,0]);
 }
});
test('loss of both command ACK and telemetry relinquishes policy ownership without OFF',async()=>{
 const c=harness();await c.handleFeedbackGap(false);c.now=1850;assert.equal(await c.handleFeedbackGap(false),true);assert.equal(c.stops,0);assert.equal(c.owned,false);
});
test('loss of both channels still unloads calibration hold',async()=>{
 const c=harness();c.controlledMode='hold';await c.handleFeedbackGap(false);c.now=1850;assert.equal(await c.handleFeedbackGap(false),true);assert.equal(c.stops,1);
});
test('a fresh frame clears the gap and requires pressing movement keys again',async()=>{
 const c=harness();await c.handleFeedbackGap(false);c.now=2000;await c.handleFeedbackGap(true);assert.equal(c.feedbackGapAt,null);assert.equal(c.stops,0);assert.equal(c.releases,2);
});
test('reading a page never acquires control and pending startup keeps its separate confirmation',async()=>{
 for(const update of [{owned:false},{sessionPending:{mode:'policy'}}]){const c=Object.assign(harness(),update);c.now=5000;await c.handleFeedbackGap(false);assert.equal(c.stops,0);assert.equal(c.releases,0);}
});
test('old session acknowledgements cannot keep a new owner alive',()=>{
 assert.match(source,/generation===actionGeneration&&owned\)lastCommandAckAt=performance.now\(\)/);
 assert.match(source,/function loseOwnership\(\)\{[^\n]*lastCommandAckAt=null/);
});
