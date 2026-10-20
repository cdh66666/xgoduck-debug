#pragma once
#include <string.h>
#include "calibration_protocol.h"

// Single-owner, nonblocking UART engine. No Arduino/Bridge/RTOS calls here.
// Port: nowUs(), available(), read(), write(bytes,size), flush().
namespace duck_calibration {
class Engine {
 public:
  Engine() { for(unsigned i=0;i<kCount;++i){cache_[i].mode=255;cache_[i].torque=255;} initResult(); }
  void begin(uint32_t bootNonce) {
    // Calling initialization again within the same epoch cannot clear sticky evidence.
    if(initialized_&&bootNonce==bootNonce_)return;
    *this=Engine();bootNonce_=bootNonce;initialized_=true;
  }
  void setReady(bool value) { ready_=value; if(value&&state_==BOOT)state_=IDLE; }
  bool motionPossible() const { return torqueMayBeOn_; }
  bool everAttempted() const { return torqueEnableEverAttempted_; }
  uint32_t lastRequest() const { return lastReceivedReqSeq_; }
  bool sessionActive() const { return session_!=0; }
  uint8_t activeId() const { return activeIndex_>=0?kIds[activeIndex_]:0; }
  uint16_t tickPositionMask() const { return tickPositionMask_; }
  bool homeReady() const { return ready_&&bootNonce_&&job_==NO_JOB&&!session_&&!torqueMayBeOn_&&state_==IDLE; }
  // Connectivity is cached real position feedback, independent of mode/value validity.
  uint16_t freshPositionMask(uint32_t now) const {
    uint16_t mask=0;for(unsigned i=0;i<kCount;++i)if(cache_[i].positionSeen&&elapsed(now,cache_[i].positionUs)<=kSourceMaxUs)mask|=(uint16_t)(1u<<i);
    return mask;
  }
  uint16_t rawWord(unsigned i) const { return cache_[i].raw; }
  int16_t decodedPosition(unsigned i) const { return cache_[i].position; }
  int16_t decodedSpeed(unsigned i) const { return cache_[i].speed; }

  static bool wireValid(const Command& c) {
    if(memcmp(c.magic,"DWC1",4)||c.version!=1||(c.op>CAPTURE_GROUP&&c.op!=PARAM_READ)||c.reserved||(c.flags&~3u)||!c.requestSeq)return false;
    if(c.op==STEP){if(!c.deltaTicks||magnitude(c.deltaTicks)>kStepMax||c.flags)return false;}
    else if(c.deltaTicks)return false;
    if(c.op==ARM&&c.flags!=3)return false;
    if((c.op==OFF_ALL||c.op==CAPTURE_ALL||c.op==CAPTURE_GROUP)&&c.flags!=1)return false;
    if((c.op==READ||c.op==DIAG||c.op==HEARTBEAT||c.op==PARAM_READ)&&c.flags)return false;
    if((c.op==READ||c.op==OFF_ALL||c.op==CAPTURE_ALL)&&c.targetId)return false;
    if((c.op==ARM||c.op==STEP||c.op==DIAG||c.op==PARAM_READ)&&indexFor(c.targetId)<0)return false;
    if(c.op==PARAM_READ&&c.session)return false;
    if(c.op==CAPTURE_GROUP&&(!groupMask(c.targetId)||c.session))return false;
    if(c.op==CANCEL&&c.targetId&&indexFor(c.targetId)<0)return false;
    return true;
  }

  template<class Port> void command(const Command& c,Port& port) {
    // A late heartbeat cannot rescue a lease that already crossed its hard deadline.
    if(ready_)checkDeadlines(port);
    const uint32_t now=port.nowUs();
    if(!wireValid(c)){reject(c,BAD_COMMAND,now);return;}
    if(!bootNonce_||c.bootNonce!=bootNonce_){reject(c,BOOT_ERROR,now);return;}
    const uint32_t step=c.requestSeq-lastReceivedReqSeq_;
    if(lastReceivedReqSeq_&&(!step||step>=0x80000000u)){reject(c,REPLAY,now);return;}
    if(c.op!=CANCEL&&c.op!=OFF_ALL&&(!c.ttlUs||c.ttlUs>kHeartbeatUs||elapsed(now,c.sourceUs)>kSourceMaxUs||elapsed(now,c.sourceUs)>c.ttlUs)){
      reject(c,STALE,now);return;
    }
    lastReceivedReqSeq_=c.requestSeq;
    if(c.op==HEARTBEAT){
      if(session_&&c.session==session_&&c.targetId==activeId()&&!expired(now,expiresUs_))lastHeartbeatUs_=now;
      return; // Heartbeat never replaces an action acknowledgement.
    }
    if(!ready_){reject(c,BUSY,now);return;}
    if(c.op==CANCEL||c.op==OFF_ALL){
      interruptQuery(port); owner_=c; setPending(c);
      captureGeneration_=0;captureMask_=0;
      const int previous=activeIndex_;
      clearSession(); stopError_=NONE; reachedPosition_=kUnknownPosition; reachedUs_=0;
      if(c.op==OFF_ALL){offAll_=true;activeIndex_=previous;beginOff(port);}
      else if(previous>=0){offAll_=false;activeIndex_=previous;owner_.targetId=kIds[previous];beginOff(port);}
      else if(torqueMayBeOn_||state_==FAULT){offAll_=true;beginOff(port);}
      else {job_=NO_JOB;state_=IDLE;complete(OK,NONE,now);}
      return;
    }
    if((c.op==ARM||c.op==STEP||c.op==CAPTURE_ALL||c.op==CAPTURE_GROUP)&&(torqueMayBeOn_||state_==FAULT)){
      reject(c,TORQUE_OFF,now);return;
    }
    if(job_!=NO_JOB||torqueMayBeOn_){reject(c,BUSY,now);return;}
    if(c.op==STEP&&(!session_||c.session!=session_||c.targetId!=activeId()||expired(now,expiresUs_))){reject(c,SESSION,now);return;}
    if(c.op==ARM&&session_){reject(c,BUSY,now);return;}
    interruptQuery(port);owner_=c;setPending(c);phase_=0;operationStartUs_=now;
    captureGeneration_=0;captureMask_=0;
    startPosition_=goal_=reachedPosition_=offPosition_=kUnknownPosition;reachedUs_=offReadUs_=0;stopError_=NONE;offAll_=false;
    if(c.op==ARM){activeIndex_=indexFor(c.targetId);job_=ARM_CHECK;state_=PREFLIGHT;}
    else if(c.op==STEP){job_=STEP_CHECK;state_=PREFLIGHT;}
    else if(c.op==PARAM_READ){
      clearSession();activeIndex_=-1;job_=PARAM_CHECK;state_=PREFLIGHT;
      memset(&parameters_,0,sizeof(parameters_));memcpy(parameters_.magic,"DPC1",4);parameters_.version=1;
      parameters_.bootNonce=bootNonce_;parameters_.requestSeq=c.requestSeq;parameters_.targetId=c.targetId;
    }
    else if(c.op==DIAG){clearSession();activeIndex_=-1;diagId_=c.targetId;job_=DIAG_CHECK;state_=PREFLIGHT;}
    else {
      const bool capture=c.op==CAPTURE_ALL||c.op==CAPTURE_GROUP;
      clearSession();activeIndex_=-1;job_=capture?CAPTURE_CHECK:READ_CHECK;state_=capture?CAPTURING:PREFLIGHT;
      if(capture){captureGeneration_=c.requestSeq;captureMask_=c.op==CAPTURE_GROUP?groupMask(c.targetId):kAllMask;captureStartUs_=captureMetadataRefreshUs_=now;captureRefreshTorque_=false;for(unsigned i=0;i<kCount;++i)if(captureMask_&(1u<<i))resetStable(cache_[i]);}
    }
  }

  template<class Port> void disarm(Port& port) {
    // Legacy DQC1 DISARM and queued emergency releases can never restore a goal.
    interruptQuery(port); clearSession(); stopError_=NONE; offAll_=true;
    captureGeneration_=0;captureMask_=0;
    owner_={};memcpy(owner_.magic,"DWC1",4);owner_.version=1;owner_.op=OFF_ALL;owner_.bootNonce=bootNonce_;
    beginOff(port);
  }

  template<class Port> void startTick(Port& port) {
    tickPositionMask_=0;
    expireCaches(port.nowUs());
    checkDeadlines(port);
    if(!ready_||queryActive_)return;
    if(torqueMayBeOn_&&state_==FAULT){
      if(elapsed(port.nowUs(),lastOffAttemptUs_)>=100000){writeTorque(port,offAll_||activeIndex_<0?254:activeId(),false);lastOffAttemptUs_=port.nowUs();}
      startQuery(port,TORQUE_READ,kAllMask,0);
      return;
    }
    if(job_==NO_JOB&&(!captureGeneration_||!captureRefreshTorque_&&elapsed(port.nowUs(),captureMetadataRefreshUs_)<200000)&&(!currentStarted_||elapsed(port.nowUs(),lastCurrentUs_)>=100000)){
      currentStarted_=true;lastCurrentUs_=port.nowUs();startQuery(port,CURRENT_READ,kAllMask,0);return;
    }
    Query kind=POSITION_READ;uint16_t mask=kAllMask;uint32_t generation=0;
    if(job_==PARAM_CHECK){kind=phase_==0?TORQUE_READ:phase_==1?PARAM21_READ:PARAM40_READ;mask=phase_==0?kAllMask:(uint16_t)(1u<<indexFor(owner_.targetId));generation=owner_.requestSeq;}
    else if(job_==ARM_CHECK){kind=phase_==0?MODE_READ:phase_==1?TORQUE_READ:phase_==2?OFFSET_READ:phase_==3?LIMIT_READ:POSITION_READ;mask=phase_<2?kAllMask:(uint16_t)(1u<<activeIndex_);generation=owner_.requestSeq;}
    else if(job_==STEP_CHECK||job_==DIAG_CHECK){
      kind=phase_==0?MODE_READ:phase_==1?TORQUE_READ:phase_==2?OFFSET_READ:phase_==3?LIMIT_READ:POSITION_READ;
      const int idx=job_==DIAG_CHECK?indexFor(owner_.targetId):activeIndex_;mask=job_==STEP_CHECK&&phase_==1?kAllMask:(uint16_t)(1u<<idx);generation=owner_.requestSeq;
    }
    else if(job_==READ_CHECK||job_==CAPTURE_CHECK){kind=phase_==0?MODE_READ:phase_==1?TORQUE_READ:POSITION_READ;generation=owner_.requestSeq;if(job_==CAPTURE_CHECK&&phase_>=2)mask=captureMask_;}
    else if(job_==MOVE_CHECK){kind=POSITION_READ;mask=(uint16_t)(1u<<activeIndex_);}
    else if(job_==OFF_CHECK){kind=TORQUE_READ;mask=kAllMask;generation=owner_.requestSeq;}
    else if(job_==OFF_POSITION_CHECK){kind=POSITION_READ;mask=(uint16_t)(1u<<activeIndex_);}
    else if(captureGeneration_&&elapsed(port.nowUs(),captureStartUs_)<kCaptureHoldUs){
      // After a successful capture, keep only a bounded read context alive for
      // Host evidence/fsync. Refresh real metadata every200ms; no writes occur.
      kind=POSITION_READ;mask=captureMask_;generation=captureGeneration_;
      if(captureRefreshTorque_){kind=TORQUE_READ;mask=kAllMask;captureRefreshTorque_=false;}
      else if(elapsed(port.nowUs(),captureMetadataRefreshUs_)>=200000){kind=MODE_READ;mask=kAllMask;captureMetadataRefreshUs_=port.nowUs();captureRefreshTorque_=true;}
    }
    else { // Background reads, no actuator writes.
      if((backgroundTick_%22)==0)kind=MODE_READ;
      else if((backgroundTick_%22)==1)kind=TORQUE_READ;
      ++backgroundTick_;
    }
    startQuery(port,kind,mask,generation);
  }

  template<class Port> void step(Port& port) {
    checkDeadlines(port);
    unsigned budget=256;
    while(budget--&&port.available()){
      const int value=port.read();if(value<0)break;
      if(queryActive_&&!expired(port.nowUs(),queryDeadlineUs_))parseByte((uint8_t)value,port.nowUs());
    }
    if(queryActive_&&(queryReceivedMask_==queryMask_||expired(port.nowUs(),queryDeadlineUs_))){
      const bool all=queryReceivedMask_==queryMask_;queryActive_=false;rxState_=rxUsed_=0;finishQuery(port,all);
    }
  }

  void snapshot(Snapshot& out,uint32_t now,uint32_t seq) const {
    memset(&out,0,sizeof(out));memcpy(out.magic,"DWR1",4);out.version=SNAPSHOT_VERSION;out.state=state_;out.flags=POLICY_LOCKED;
    if(ready_&&bootNonce_)out.flags|=BOOT_READY;
    if(session_)out.flags|=SESSION_ACTIVE;
    if(torqueMayBeOn_)out.flags|=TORQUE_MAY_BE_ON;
    if(job_==CAPTURE_CHECK)out.flags|=CAPTURE_BUSY;
    if(!torqueMayBeOn_&&state_!=FAULT&&allTorqueOff(now))out.flags|=ALL_TORQUE_OFF_VERIFIED;
    if(torqueEnableEverAttempted_)out.flags|=TORQUE_ENABLE_EVER_ATTEMPTED;
    out.activeId=activeId();out.bootNonce=bootNonce_;out.stateSeq=seq;out.sampleUs=now;out.operationAckSeq=ackSeq_;
    out.session=session_;out.expiresUs=expiresUs_;out.lastReceivedReqSeq=lastReceivedReqSeq_;out.anchor=anchor_;out.goal=goal_;
    out.result=ackResult_;out.op=ackOp_;out.error=ackError_;out.diagId=diagId_;out.diagMin=out.diagMax=UINT16_MAX;
    const int diag=indexFor(diagId_);
    if(diag>=0&&cache_[diag].offsetSeen&&cache_[diag].limitsSeen){
      out.diagOffset=signedOffset(cache_[diag].offsetRaw);out.diagMin=cache_[diag].minimum;out.diagMax=cache_[diag].maximum;
      if(elapsed(now,cache_[diag].offsetUs)<=kMetadataMaxUs&&elapsed(now,cache_[diag].limitsUs)<=kMetadataMaxUs&&!(cache_[diag].offsetRaw&0xf000u)&&cache_[diag].minimum<=cache_[diag].maximum&&cache_[diag].maximum<=4095)out.flags|=DIAG_READY;
    }
    for(unsigned i=0;i<kCount;++i){
      const Cache& c=cache_[i];Row& row=out.rows[i];row.mode=c.modeSeen?c.mode:255;row.torque=c.torqueSeen?c.torque:255;
      row.positionAgeMs=c.positionSeen?ageMs(now,c.positionUs):UINT16_MAX;
      row.metadataAgeMs=c.modeSeen&&c.torqueSeen?ageMs(now,older(c.modeUs,c.torqueUs,now)):UINT16_MAX;
      if(c.positionSeen){row.rawWord=c.raw;row.signedSteps=c.position;row.voltageRaw=c.voltage;row.temperature=c.temperature;}
      else row.signedSteps=kUnknownPosition;
      const uint16_t bit=(uint16_t)(1u<<i);
      if(positionFresh(c,now))out.positionMask|=bit;
      if(metadataFresh(c,now))out.metadataMask|=bit;
      if(stable(c,now))out.stableMask|=bit;
      if(captureValid(i,now))out.captureMask|=bit;
    }
  }
  bool takeResult(ActionResult& out) { if(!resultReady_)return false;out=lastResult_;resultReady_=false;return true; }
  void parameterSnapshot(ParameterSnapshot& out) const {out=parameters_;}
  void currentSnapshot(CurrentSnapshot& out,uint32_t now,uint32_t seq) const {
    memset(&out,0,sizeof(out));memcpy(out.magic,"DIC1",4);out.version=1;out.bootNonce=bootNonce_;out.stateSeq=seq;out.sampleUs=now;
    for(unsigned index=0;index<kCount;++index){const Cache& cell=cache_[index];out.rawWords[index]=cell.currentRaw;out.agesMs[index]=cell.currentSeen?ageMs(now,cell.currentUs):UINT16_MAX;if(cell.currentSeen&&elapsed(now,cell.currentUs)<=kSourceMaxUs)out.validMask|=1u<<index;}
  }

 private:
  enum Query:uint8_t { POSITION_READ,MODE_READ,TORQUE_READ,OFFSET_READ,LIMIT_READ,PARAM21_READ,PARAM40_READ,CURRENT_READ };
  enum Job:uint8_t { NO_JOB,READ_CHECK,ARM_CHECK,STEP_CHECK,CAPTURE_CHECK,DIAG_CHECK,MOVE_CHECK,OFF_CHECK,OFF_POSITION_CHECK,PARAM_CHECK };
  struct Cache {
    uint16_t currentRaw=0;uint32_t currentUs=0;bool currentSeen=false;
    uint16_t raw=0,offsetRaw=0,minimum=0,maximum=0;int16_t position=0,speed=0,stableMin=0,stableMax=0;
    uint8_t mode=255,torque=255,voltage=0,temperature=0;uint16_t stableCount=0;
    bool positionSeen=false,modeSeen=false,torqueSeen=false,offsetSeen=false,limitsSeen=false;
    uint32_t positionUs=0,modeUs=0,torqueUs=0,offsetUs=0,limitsUs=0,stableStartUs=0,modeGeneration=0,torqueGeneration=0;
  } cache_[kCount];
  uint32_t bootNonce_=0,session_=0,expiresUs_=0,lastHeartbeatUs_=0,lastReceivedReqSeq_=0,sessionCounter_=0;
  uint32_t operationStartUs_=0,goalWriteUs_=0,moveDeadlineUs_=0,offStartUs_=0,lastOffAttemptUs_=0;
  uint32_t captureGeneration_=0,captureStartUs_=0,captureMetadataRefreshUs_=0,queryGeneration_=0,queryDeadlineUs_=0,backgroundTick_=0;
  uint32_t ackSeq_=0,reachedUs_=0,offReadUs_=0;
  int activeIndex_=-1;int16_t anchor_=kUnknownPosition,goal_=kUnknownPosition,startPosition_=kUnknownPosition,reachedPosition_=kUnknownPosition,offPosition_=kUnknownPosition;
  bool ready_=false,torqueMayBeOn_=false,offAll_=false,queryActive_=false,resultReady_=false,captureRefreshTorque_=false;
  bool initialized_=false,torqueEnableEverAttempted_=false;
  uint8_t state_=BOOT,ackResult_=PENDING,ackOp_=READ,ackError_=NONE,diagId_=0,phase_=0,stopError_=NONE;
  Job job_=NO_JOB;Query queryKind_=POSITION_READ;
  uint16_t queryMask_=0,queryReceivedMask_=0,tickPositionMask_=0,captureMask_=0;
  uint8_t rxState_=0,rxUsed_=0,rxLength_=0,rx_[20]={};Command owner_={};ActionResult lastResult_={};
  ParameterSnapshot parameters_={};
  uint32_t lastCurrentUs_=0;bool currentStarted_=false;
  static uint16_t word(const uint8_t* p) { return (uint16_t)p[0]|((uint16_t)p[1]<<8); }
  static uint16_t ageMs(uint32_t now,uint32_t timestamp) {const uint32_t age=elapsed(now,timestamp)/1000;return age>=UINT16_MAX?UINT16_MAX:(uint16_t)age;}
  static uint32_t older(uint32_t a,uint32_t b,uint32_t now){return elapsed(now,a)>elapsed(now,b)?a:b;}
  // FEETECH STS memory table: position modes 0 and 4 (integral-limit row).
  // https://ftservo.github.io/ftservo-wiki/reference/parameter/memory-sts/
  static bool positionMode(uint8_t mode){return mode==0||mode==4;}
  static bool singleTurnPosition(const Cache& c){return c.raw<=4095&&c.position>=0&&c.position<=4095&&c.raw==(uint16_t)c.position;}
  // Capture stores native evidence only. Motion keeps the single-turn gate above.
  static bool capturePosition(const Cache& c){return c.mode==0?singleTurnPosition(c):c.mode==4&&c.raw!=0x8000u&&c.position==signedPosition(c.raw);}
  static bool positionFresh(const Cache& c,uint32_t now){return c.positionSeen&&elapsed(now,c.positionUs)<=kSourceMaxUs;}
  static bool metadataFresh(const Cache& c,uint32_t now){return c.modeSeen&&c.mode!=255&&c.torqueSeen&&c.torque<=1&&elapsed(now,c.modeUs)<=kMetadataMaxUs&&elapsed(now,c.torqueUs)<=kMetadataMaxUs;}
  static void resetStable(Cache& c){c.stableCount=0;c.stableStartUs=0;}
  static bool stable(const Cache& c,uint32_t now){return positionFresh(c,now)&&c.stableCount>=3&&elapsed(c.positionUs,c.stableStartUs)>=100000&&magnitude(c.stableMax-c.stableMin)<=3;}
  bool captureValid(unsigned i,uint32_t now) const {
    const Cache& c=cache_[i];return (captureMask_&(1u<<i))&&!torqueMayBeOn_&&state_!=FAULT&&captureMetadataFresh(now)&&captureGeneration_&&elapsed(now,captureStartUs_)<=kCaptureHoldUs&&c.modeGeneration==captureGeneration_&&c.torqueGeneration==captureGeneration_
      &&metadataFresh(c,now)&&positionMode(c.mode)&&c.torque==0&&capturePosition(c)&&stable(c,now);
  }
  bool allTorqueOff(uint32_t now) const {for(unsigned i=0;i<kCount;++i)if(!cache_[i].torqueSeen||elapsed(now,cache_[i].torqueUs)>kMetadataMaxUs||cache_[i].torque!=0)return false;return true;}
  bool captureMetadataFresh(uint32_t now) const {for(unsigned i=0;i<kCount;++i){const Cache& c=cache_[i];if(!metadataFresh(c,now)||c.modeGeneration!=captureGeneration_||c.torqueGeneration!=captureGeneration_||c.torque!=0)return false;}return true;}
  void clearSession(){session_=expiresUs_=lastHeartbeatUs_=0;}
  void expireCaches(uint32_t now){
    for(unsigned i=0;i<kCount;++i){Cache& c=cache_[i];
      if(c.positionSeen&&elapsed(now,c.positionUs)>kSourceMaxUs){c.positionSeen=false;resetStable(c);}
      if(c.modeSeen&&elapsed(now,c.modeUs)>kMetadataMaxUs)c.modeSeen=false;
      if(c.torqueSeen&&elapsed(now,c.torqueUs)>kMetadataMaxUs)c.torqueSeen=false;
      if(c.offsetSeen&&elapsed(now,c.offsetUs)>kMetadataMaxUs)c.offsetSeen=false;
      if(c.limitsSeen&&elapsed(now,c.limitsUs)>kMetadataMaxUs)c.limitsSeen=false;
    }
    if(captureGeneration_&&elapsed(now,captureStartUs_)>kCaptureHoldUs){captureGeneration_=0;captureMask_=0;}
  }
  void initResult(){memcpy(lastResult_.magic,"DWA1",4);lastResult_.version=1;lastResult_.anchor=lastResult_.start=lastResult_.goal=lastResult_.reachedPosition=lastResult_.offPosition=kUnknownPosition;}
  void setPending(const Command& c){ackSeq_=c.requestSeq;ackResult_=PENDING;ackOp_=c.op;ackError_=NONE;}
  void reject(const Command& c,uint8_t error,uint32_t now){if(c.op==HEARTBEAT)return;Command saved=owner_;owner_=c;complete(ERROR,error,now);owner_=saved;}
  void complete(uint8_t result,uint8_t error,uint32_t now){
    if(owner_.op==PARAM_READ&&parameters_.requestSeq==owner_.requestSeq){parameters_.result=result;parameters_.error=error;parameters_.sampleUs=now;}
    if(result==ERROR&&(owner_.op==CAPTURE_ALL||owner_.op==CAPTURE_GROUP)){captureGeneration_=0;captureMask_=0;}
    ackSeq_=owner_.requestSeq;ackResult_=result;ackOp_=owner_.op;ackError_=error;
    memset(&lastResult_,0,sizeof(lastResult_));initResult();lastResult_.op=owner_.op;lastResult_.targetId=owner_.targetId;lastResult_.result=result;
    lastResult_.bootNonce=bootNonce_;lastResult_.requestSeq=owner_.requestSeq;lastResult_.session=owner_.op==ARM?session_:owner_.session;lastResult_.sampleUs=now;
    lastResult_.anchor=anchor_;lastResult_.start=startPosition_;lastResult_.goal=goal_;lastResult_.reachedPosition=reachedPosition_;lastResult_.offPosition=offPosition_;lastResult_.deltaTicks=owner_.deltaTicks;
    lastResult_.reachedUs=reachedUs_;lastResult_.offReadUs=offReadUs_;lastResult_.error=error;
    const int idx=indexFor(owner_.targetId);if(idx>=0){lastResult_.voltageRaw=cache_[idx].voltage;lastResult_.temperature=cache_[idx].temperature;}
    resultReady_=true;
  }
  template<class Port> void interruptQuery(Port& port){queryActive_=false;rxState_=rxUsed_=0;unsigned n=256;while(n--&&port.available())port.read();}
  template<class Port> bool send(Port& port,uint8_t id,uint8_t instruction,const uint8_t* data,uint8_t length){
    uint8_t out[24]={255,255,id,(uint8_t)(length+2),instruction};uint8_t sum=(uint8_t)(id+out[3]+instruction);
    for(uint8_t i=0;i<length;++i){out[5+i]=data[i];sum=(uint8_t)(sum+data[i]);}out[5+length]=(uint8_t)~sum;
    const bool full=port.write(out,length+6)==(size_t)(length+6);port.flush();return full;
  }
  template<class Port> bool writeTorque(Port& port,uint8_t id,bool on){
    // This records an attempt, before UART write/flush and even if TX fails.
    if(on)torqueEnableEverAttempted_=true;
    // A write is never a readback: old torque samples cannot certify this new state.
    for(unsigned i=0;i<kCount;++i)if(id==254||id==kIds[i])cache_[i].torqueSeen=false;
    const uint8_t data[2]={40,(uint8_t)(on?1:0)};return send(port,id,3,data,2);
  }
  template<class Port> bool writeGoal(Port& port,uint8_t id,uint16_t goal){const uint8_t data[7]={42,(uint8_t)goal,(uint8_t)(goal>>8),0,0,100,0};return send(port,id,3,data,7);}
  template<class Port> void startQuery(Port& port,Query kind,uint16_t mask,uint32_t generation){
    unsigned drain=256;while(drain--&&port.available())port.read();rxState_=rxUsed_=0;
    queryKind_=kind;queryMask_=mask;queryReceivedMask_=0;queryGeneration_=generation;
    const uint8_t address=kind==CURRENT_READ?69:kind==POSITION_READ?56:kind==MODE_READ?33:kind==TORQUE_READ||kind==PARAM40_READ?40:kind==OFFSET_READ?31:kind==PARAM21_READ?21:9;
    const uint8_t length=kind==POSITION_READ?8:kind==OFFSET_READ||kind==CURRENT_READ?2:kind==LIMIT_READ?4:kind==PARAM21_READ?7:kind==PARAM40_READ?12:1;
    queryDeadlineUs_=port.nowUs()+kReadWindowUs;queryActive_=true;bool sent=false;
    if(mask&&(mask&(mask-1u))){uint8_t bytes[17]={address,length};uint8_t used=2;for(unsigned i=0;i<kCount;++i)if(mask&(1u<<i))bytes[used++]=kIds[i];sent=send(port,254,0x82,bytes,used);}
    else {int idx=0;while(idx<kCount&&!(mask&(1u<<idx)))++idx;const uint8_t bytes[2]={address,length};sent=idx<kCount&&send(port,kIds[idx],2,bytes,2);}
    if(!sent)queryDeadlineUs_=port.nowUs();
  }
  void parseByte(uint8_t value,uint32_t now){
    if(rxState_==0){if(value==255){rx_[0]=value;rxState_=1;}return;}
    if(rxState_==1){rxState_=value==255?2:0;rx_[1]=value;return;}
    if(rxState_==2){if(value==255)return;rx_[2]=value;rxState_=3;return;}
    if(rxState_==3){
      const uint8_t expected=queryKind_==POSITION_READ?10:queryKind_==OFFSET_READ||queryKind_==CURRENT_READ?4:queryKind_==LIMIT_READ?6:queryKind_==PARAM21_READ?9:queryKind_==PARAM40_READ?14:3;
      if(value!=expected){rxState_=value==255?1:0;return;}rx_[3]=value;rxLength_=value;rxUsed_=0;rxState_=4;return;
    }
    if((unsigned)(4+rxUsed_)>=sizeof(rx_)){rxState_=0;return;}rx_[4+rxUsed_++]=value;if(rxUsed_<rxLength_)return;rxState_=0;
    uint8_t sum=0;for(unsigned i=2;i<(unsigned)(3+rxLength_);++i)sum=(uint8_t)(sum+rx_[i]);
    const int idx=indexFor(rx_[2]);if((uint8_t)~sum!=rx_[3+rxLength_]||rx_[4]||idx<0||!(queryMask_&(1u<<idx)))return;
    const uint16_t bit=(uint16_t)(1u<<idx);if(queryReceivedMask_&bit)return;queryReceivedMask_|=bit;Cache& c=cache_[idx];
    if(queryKind_==CURRENT_READ){c.currentRaw=word(rx_+5);c.currentSeen=true;c.currentUs=now;}
    else if(queryKind_==POSITION_READ){
      c.raw=word(rx_+5);c.position=signedPosition(c.raw);c.speed=signedPosition(word(rx_+7));c.voltage=rx_[11];c.temperature=rx_[12];c.positionSeen=true;c.positionUs=now;tickPositionMask_|=bit;
      if(!c.stableCount){c.stableStartUs=now;c.stableMin=c.stableMax=c.position;c.stableCount=1;}
      else {if(c.position<c.stableMin)c.stableMin=c.position;if(c.position>c.stableMax)c.stableMax=c.position;
        if(magnitude(c.stableMax-c.stableMin)>3){c.stableStartUs=now;c.stableMin=c.stableMax=c.position;c.stableCount=1;}else if(c.stableCount<65535)++c.stableCount;}
    } else if(queryKind_==MODE_READ){c.mode=rx_[5];c.modeSeen=true;c.modeUs=now;c.modeGeneration=queryGeneration_;}
    else if(queryKind_==TORQUE_READ){c.torque=rx_[5];c.torqueSeen=true;c.torqueUs=now;c.torqueGeneration=queryGeneration_;}
    else if(queryKind_==OFFSET_READ){c.offsetRaw=word(rx_+5);c.offsetSeen=true;c.offsetUs=now;}
    else if(queryKind_==PARAM21_READ){memcpy(parameters_.parameter21,rx_+5,7);parameters_.parameter21Us=now;}
    else if(queryKind_==PARAM40_READ){memcpy(parameters_.parameter40,rx_+5,12);parameters_.parameter40Us=now;}
    else {c.minimum=word(rx_+5);c.maximum=word(rx_+7);c.limitsSeen=true;c.limitsUs=now;}
  }
  uint8_t motionError(uint32_t now) const {
    if(activeIndex_<0)return BAD_ID;const Cache& c=cache_[activeIndex_];
    if(!positionFresh(c,now)||!metadataFresh(c,now))return FEEDBACK;
    if(!positionMode(c.mode))return MODE;if(!singleTurnPosition(c))return POSITION;if(c.torque!=0)return TORQUE_OFF;
    if(c.voltage<40||c.voltage>84)return VOLTAGE;if(c.temperature>=65)return TEMPERATURE;
    if(!c.offsetSeen||!c.limitsSeen||elapsed(now,c.offsetUs)>kMetadataMaxUs||elapsed(now,c.limitsUs)>kMetadataMaxUs||(c.offsetRaw&0xf000u))return DIAG_ERROR;
    if(c.minimum>c.maximum||c.maximum>4095||c.position<c.minimum||c.position>c.maximum)return RANGE;
    return NONE;
  }
  template<class Port> void beginOff(Port& port){
    interruptQuery(port);state_=VERIFY_OFF;job_=OFF_CHECK;phase_=0;offStartUs_=lastOffAttemptUs_=port.nowUs();
    const bool sent=writeTorque(port,offAll_?254:activeId(),false);if(!sent)stopError_=TORQUE_OFF;
  }
  template<class Port> void failMotion(Port& port,uint8_t error){stopError_=error;clearSession();beginOff(port);}
  template<class Port> void checkDeadlines(Port& port){
    const uint32_t now=port.nowUs();
    if(session_&&(expired(now,expiresUs_)||elapsed(now,lastHeartbeatUs_)>=kHeartbeatUs)){
      owner_.targetId=activeId();failMotion(port,expired(now,expiresUs_)?SESSION:STALE);return;
    }
    if(job_==MOVE_CHECK&&expired(now,moveDeadlineUs_)){failMotion(port,MOVE_TIMEOUT);return;}
    if(job_==CAPTURE_CHECK&&elapsed(now,operationStartUs_)>=1500000){job_=NO_JOB;state_=IDLE;complete(ERROR,UNSTABLE,now);}
  }
  template<class Port> void finishQuery(Port& port,bool all){
    const uint32_t now=port.nowUs();
    if(queryKind_==CURRENT_READ)return;
    if(job_==NO_JOB){
      if(torqueMayBeOn_&&queryKind_==TORQUE_READ&&all&&allTorqueOff(now)){
        torqueMayBeOn_=false;activeIndex_=-1;state_=IDLE;
      }return;
    }
    if(job_==MOVE_CHECK){
      const Cache& c=cache_[activeIndex_];if(!all||!positionFresh(c,now)){failMotion(port,FEEDBACK);return;}
      if(!singleTurnPosition(c)||magnitude(c.position-startPosition_)>kStepMax+8){failMotion(port,POSITION);return;}
      if(c.voltage<40||c.voltage>84){failMotion(port,VOLTAGE);return;}if(c.temperature>=65){failMotion(port,TEMPERATURE);return;}
      const int tolerance=magnitude(owner_.deltaTicks)<=3?1:(magnitude(owner_.deltaTicks)/3<3?magnitude(owner_.deltaTicks)/3:3);
      if(elapsed(c.positionUs,goalWriteUs_)<0x80000000u&&c.positionUs!=goalWriteUs_&&c.position!=startPosition_&&magnitude(c.position-goal_)<=tolerance){reachedPosition_=c.position;reachedUs_=c.positionUs;beginOff(port);}
      return;
    }
    if(job_==OFF_CHECK){
      bool off=all;for(unsigned i=0;i<kCount;++i)if((queryMask_&(1u<<i))&&cache_[i].torque!=0)off=false;
      if(off){torqueMayBeOn_=false;if(offAll_||activeIndex_<0){job_=NO_JOB;activeIndex_=-1;state_=IDLE;complete(stopError_?ERROR:OK,stopError_,now);}else job_=OFF_POSITION_CHECK;}
      else if(elapsed(now,offStartUs_)>=150000){clearSession();job_=NO_JOB;state_=FAULT;torqueMayBeOn_=true;complete(ERROR,TORQUE_OFF,now);}
      else {writeTorque(port,offAll_?254:activeId(),false);lastOffAttemptUs_=now;}
      return;
    }
    if(job_==OFF_POSITION_CHECK){
      if(all){offPosition_=cache_[activeIndex_].position;offReadUs_=cache_[activeIndex_].positionUs;}
      job_=NO_JOB;if(stopError_||!all){clearSession();activeIndex_=-1;state_=IDLE;complete(ERROR,stopError_?stopError_:FEEDBACK,now);}
      else if(owner_.op==STEP){state_=ARMED;complete(OK,NONE,now);}
      else {clearSession();activeIndex_=-1;state_=IDLE;complete(OK,NONE,now);}return;
    }
    if(!all){job_=NO_JOB;if(owner_.op==ARM)activeIndex_=-1;state_=session_?ARMED:IDLE;complete(ERROR,INCOMPLETE,now);return;}
    if(job_==PARAM_CHECK){
      if(!allTorqueOff(now)){job_=NO_JOB;state_=IDLE;complete(ERROR,TORQUE_OFF,now);return;}
      if(++phase_<3)return;
      job_=NO_JOB;state_=IDLE;complete(OK,NONE,now);return;
    }
    if(job_==READ_CHECK||job_==CAPTURE_CHECK){
      if(phase_<2){++phase_;return;}
      if(job_==READ_CHECK){job_=NO_JOB;state_=IDLE;complete(OK,NONE,now);return;}
      // Every capture rereads the full bus metadata for this request. Other
      // groups may have invalid positions; their torque-off must still be real.
      for(unsigned i=0;i<kCount;++i){const Cache& c=cache_[i];if(!metadataFresh(c,now)||c.modeGeneration!=captureGeneration_||c.torqueGeneration!=captureGeneration_||c.torque!=0){diagId_=kIds[i];job_=NO_JOB;state_=IDLE;complete(ERROR,TORQUE_OFF,now);return;}}
      for(unsigned i=0;i<kCount;++i){
        if(!(captureMask_&(1u<<i)))continue;
        const Cache& c=cache_[i];uint8_t error=NONE;
        if(!positionMode(c.mode))error=MODE;else if(c.torque!=0)error=TORQUE_OFF;else if(!capturePosition(c))error=POSITION;
        if(error){diagId_=kIds[i];job_=NO_JOB;state_=IDLE;complete(ERROR,error,now);return;}
      }
      uint16_t mask=0;for(unsigned i=0;i<kCount;++i)if(captureValid(i,now))mask|=(1u<<i);
      if(mask==captureMask_&&captureMask_){job_=NO_JOB;state_=IDLE;complete(OK,NONE,now);}return;
    }
    if(++phase_<5)return;
    if(job_==DIAG_CHECK){job_=NO_JOB;state_=IDLE;complete(OK,NONE,now);return;}
    const uint8_t error=motionError(now);if(error){job_=NO_JOB;if(owner_.op==ARM)activeIndex_=-1;state_=session_?ARMED:IDLE;complete(ERROR,error,now);return;}
    if(torqueMayBeOn_||state_==FAULT||!allTorqueOff(now)){
      job_=NO_JOB;if(owner_.op==ARM)activeIndex_=-1;state_=session_?ARMED:IDLE;complete(ERROR,TORQUE_OFF,now);return;
    }
    if(job_==ARM_CHECK){
      if(!allTorqueOff(now)){job_=NO_JOB;activeIndex_=-1;state_=IDLE;complete(ERROR,TORQUE_OFF,now);return;}
      anchor_=cache_[activeIndex_].position;startPosition_=goal_=anchor_;diagId_=activeId();
      ++sessionCounter_;session_=bootNonce_^sessionCounter_^now;if(!session_)session_=sessionCounter_?sessionCounter_:1;
      expiresUs_=now+kSessionUs;lastHeartbeatUs_=now;job_=NO_JOB;state_=ARMED;complete(OK,NONE,now);return;
    }
    const Cache& c=cache_[activeIndex_];startPosition_=c.position;const int32_t target=(int32_t)c.position+owner_.deltaTicks;
    if(target<0||target>4095){job_=NO_JOB;state_=ARMED;complete(ERROR,POSITION,now);return;}
    if(magnitude(target-anchor_)>kOffsetMax||magnitude(c.position-anchor_)>kOffsetMax||target<c.minimum||target>c.maximum){job_=NO_JOB;state_=ARMED;complete(ERROR,RANGE,now);return;}
    if(!session_||expired(now,expiresUs_)||elapsed(now,lastHeartbeatUs_)>=kHeartbeatUs){failMotion(port,SESSION);return;}
    if(elapsed(now,owner_.sourceUs)>kSourceMaxUs||elapsed(now,owner_.sourceUs)>owner_.ttlUs){job_=NO_JOB;state_=ARMED;complete(ERROR,STALE,now);return;}
    goal_=(int16_t)target;
    if(!writeTorque(port,activeId(),false)){failMotion(port,FEEDBACK);return;}
    if(expired(port.nowUs(),expiresUs_)||elapsed(port.nowUs(),lastHeartbeatUs_)>=kHeartbeatUs||elapsed(port.nowUs(),owner_.sourceUs)>kSourceMaxUs||elapsed(port.nowUs(),owner_.sourceUs)>owner_.ttlUs){failMotion(port,STALE);return;}
    if(!writeGoal(port,activeId(),(uint16_t)target)){failMotion(port,FEEDBACK);return;}
    if(expired(port.nowUs(),expiresUs_)||elapsed(port.nowUs(),lastHeartbeatUs_)>=kHeartbeatUs||elapsed(port.nowUs(),owner_.sourceUs)>kSourceMaxUs||elapsed(port.nowUs(),owner_.sourceUs)>owner_.ttlUs){failMotion(port,STALE);return;}
    torqueMayBeOn_=true;
    if(!writeTorque(port,activeId(),true)){failMotion(port,TORQUE_OFF);return;}
    goalWriteUs_=port.nowUs();moveDeadlineUs_=goalWriteUs_+kMoveUs;job_=MOVE_CHECK;state_=MOVING;
  }
};

// Exclusive UART owner. No other engine may issue writes while HOME owns the bus.
// Software trajectory bounds are NOT a claim of mode4 hardware speed/torque limits.
class HomeEngine {
 public:
  void begin(uint32_t boot){
    if(boot_==boot&&initialized_)return;
    initialized_=true;boot_=boot;phase_=HOME_OFF;result_=OK;error_=NONE;
    nonce_=deadline_=ack_=lastRequest_=0;possible_=ever_=false;queryActive_=false;
    offMask_=commandedMask_=completedMask_=steps_=tickMask_=0;
    sequential_=selected_=sequencePolling_=false;selectionSeq_=captureSeq_=captureStartUs_=captureUs_=heldUs_=0;
    currentStarted_=false;lastCurrentUs_=0;
    active_=0;stage_=cycle_=faultId_=faultStage_=pendingFaultId_=0;
    for(unsigned i=0;i<kCount;++i){cells_[i]=Cell();targets_[i]=goals_[i]=prime_[i]=0;modes_[i]=255;trackSince_[i]=0;}
    memset(sha_,0,32);
  }
  bool ownsBus()const{return (phase_>=HOME_PREPARING&&phase_<=HOME_RELEASING)||(phase_==HOME_FAULT&&possible_);}
  bool possibleMotion()const{return possible_;}
  bool everAttempted()const{return ever_;}
  uint32_t lastRequest()const{return lastRequest_;}
  uint16_t tickPositionMask()const{return tickMask_;}
  uint16_t rawWord(unsigned i)const{return cells_[i].raw;}
  int16_t position(unsigned i)const{return cells_[i].position;}
  int16_t speed(unsigned i)const{return cells_[i].speed;}
  uint16_t freshPositionMask(uint32_t now)const{uint16_t m=0;for(unsigned i=0;i<kCount;++i)if(posFresh(cells_[i],now))m|=1u<<i;return m;}
  static bool wireValid(const HomeCommand& c){
    if(memcmp(c.magic,"DWH1",4)||c.version!=HOME_CONTRACT_VERSION||c.flags!=3||c.reserved||c.reservedTail[0]||c.reservedTail[1]||c.reservedTail[2]||!c.bootNonce||!c.requestSeq||!c.session||!c.ttlUs||c.ttlUs>kHeartbeatUs)return false;
    uint8_t any=0;for(unsigned i=0;i<32;++i)any|=c.nativeSha256[i];if(!any)return false;
    for(unsigned i=0;i<kCount;++i)if((c.modes[i]!=0&&c.modes[i]!=4)||c.targets[i]==kUnknownPosition||(c.modes[i]==0&&(c.targets[i]<0||c.targets[i]>4095)))return false;
    return true;
  }
  static bool controlValid(const Command& c){return !memcmp(c.magic,"DWC1",4)&&c.version==1&&c.targetId==0&&!c.reserved&&!c.deltaTicks&&c.requestSeq&&c.bootNonce&&c.ttlUs&&c.ttlUs<=kHeartbeatUs&&((c.op==HOME_RUN||c.op==HEARTBEAT||c.op==CANCEL)&&c.flags==0||c.op==OFF_ALL&&c.flags==1);}
  static bool sequenceValid(const SequenceCommand& c){
    if(memcmp(c.magic,"DZC1",4)||c.version!=1||c.op>SEQ_STOP||c.reserved||!c.bootNonce||!c.requestSeq||!c.ttlUs||c.ttlUs>kHeartbeatUs)return false;
    if(c.op==SEQ_BEGIN||c.op==SEQ_HOLD){if(c.flags!=3)return false;}else if(c.flags!=(c.op==SEQ_STOP?1:0))return false;
    if(c.op!=SEQ_STOP&&!c.session)return false;
    if(c.op==SEQ_SELECT||c.op==SEQ_HOLD){if(indexFor(c.targetId)<0)return false;}else if(c.targetId)return false;
    if(c.op==SEQ_HOLD)return c.selectionSeq&&c.expectedPosition!=kUnknownPosition&&(c.expectedMode==0||c.expectedMode==4);
    return !c.selectionSeq&&!c.expectedPosition&&!c.expectedMode;
  }
  template<class Port>void sequence(const SequenceCommand& c,Port& p){
    checkDeadlines(p);const uint32_t now=p.nowUs();
    if(!sequenceValid(c)||c.bootNonce!=boot_)return;
    if(lastRequest_&&(!elapsed(c.requestSeq,lastRequest_)||elapsed(c.requestSeq,lastRequest_)>=0x80000000u))return;
    if(c.op==SEQ_STOP){lastRequest_=ack_=c.requestSeq;release(p,NONE);return;}
    if(elapsed(now,c.sourceUs)>kSourceMaxUs||elapsed(now,c.sourceUs)>c.ttlUs)return;
    if(c.op==SEQ_BEGIN){
      if(ownsBus()||possible_)return;
      interrupt(p);lastRequest_=ack_=c.requestSeq;nonce_=c.session;lastBeat_=now;deadline_=now+2000000;
      sequential_=true;selected_=sequencePolling_=false;phase_=HOME_PREPARING;result_=PENDING;error_=NONE;
      stage_=cycle_=active_=faultId_=faultStage_=pendingFaultId_=0;
      steps_=offMask_=commandedMask_=completedMask_=0;generation_=c.requestSeq;
      selectionSeq_=captureSeq_=captureStartUs_=captureUs_=heldUs_=0;capturePosition_=kUnknownPosition;captureMode_=255;
      for(unsigned index=0;index<kCount;++index){cells_[index]=Cell();targets_[index]=goals_[index]=prime_[index]=0;modes_[index]=255;trackSince_[index]=0;}
      memset(sha_,0,sizeof(sha_));return;
    }
    if(!sequential_||!nonce_||c.session!=nonce_)return;
    lastRequest_=c.requestSeq;
    if(c.op==SEQ_HEARTBEAT){lastBeat_=now;return;}
    if(c.op==SEQ_SELECT){
      if((phase_!=HOME_PREPARED&&phase_!=HOME_HOLDING)||(completedMask_&(1u<<indexFor(c.targetId))))return;
      interrupt(p);ack_=selectionSeq_=c.requestSeq;active_=indexFor(c.targetId);selected_=true;
      captureSeq_=captureStartUs_=captureUs_=heldUs_=0;capturePosition_=kUnknownPosition;captureMode_=255;
      phase_=HOME_PREPARED;result_=OK;error_=NONE;cycle_=0;return;
    }
    if(c.op==SEQ_HOLD){
      if(phase_!=HOME_PREPARED||!selected_||c.targetId!=kIds[active_]||c.selectionSeq!=selectionSeq_||c.expectedMode!=modes_[active_]){ack_=c.requestSeq;release(p,SESSION);return;}
      interrupt(p);ack_=captureSeq_=c.requestSeq;generation_=c.requestSeq;
      expectedPosition_=c.expectedPosition;captureStartUs_=captureUs_=heldUs_=0;sequencePolling_=false;
      startJoint(now);jointBudgetUs_=2000000;result_=PENDING;error_=NONE;
    }
  }
  template<class Port>void prepare(const HomeCommand& c,Port& p){
    checkDeadlines(p);const uint32_t now=p.nowUs();
    if(!wireValid(c)||c.bootNonce!=boot_||elapsed(now,c.sourceUs)>kSourceMaxUs||elapsed(now,c.sourceUs)>c.ttlUs||ownsBus()||possible_)return;
    if(lastRequest_&&(!elapsed(c.requestSeq,lastRequest_)||elapsed(c.requestSeq,lastRequest_)>=0x80000000u))return;
    interrupt(p);lastRequest_=ack_=c.requestSeq;nonce_=c.session;deadline_=now+kSessionUs;lastBeat_=now;
    sequential_=selected_=sequencePolling_=false;
    phase_=HOME_PREPARING;result_=PENDING;error_=NONE;stage_=cycle_=active_=0;
    faultId_=faultStage_=pendingFaultId_=0;steps_=offMask_=commandedMask_=completedMask_=0;generation_=c.requestSeq;
    for(unsigned i=0;i<kCount;++i){targets_[i]=c.targets[i];modes_[i]=c.modes[i];cells_[i].goalSeen=false;trackSince_[i]=0;}
    memcpy(sha_,c.nativeSha256,32);
  }
  template<class Port>void control(const Command& c,Port& p){
    checkDeadlines(p);const uint32_t now=p.nowUs();
    if(!controlValid(c)||c.bootNonce!=boot_)return;
    if(lastRequest_&&(!elapsed(c.requestSeq,lastRequest_)||elapsed(c.requestSeq,lastRequest_)>=0x80000000u))return;
    lastRequest_=c.requestSeq;
    if(c.op==CANCEL||c.op==OFF_ALL){ack_=c.requestSeq;release(p,NONE);return;}
    if(sequential_)return;
    if(!nonce_||c.session!=nonce_||(phase_!=HOME_HOLDING&&expired(now,deadline_))||elapsed(now,c.sourceUs)>kSourceMaxUs||elapsed(now,c.sourceUs)>c.ttlUs)return;
    if(c.op==HEARTBEAT){lastBeat_=now;return;}
    if(c.op==HOME_RUN&&phase_==HOME_PREPARED){
      interrupt(p);ack_=c.requestSeq;generation_=c.requestSeq;deadline_=now+kHomeMotionUs;
      active_=0;startJoint(now);result_=PENDING;error_=NONE;
    }
  }
  template<class Port>void stop(Port& p,uint32_t request=0){if(request)ack_=request;release(p,NONE);}
  template<class Port>void startTick(Port& p){
    tickMask_=0;checkDeadlines(p);if(!ownsBus()||queryActive_)return;
    if(phase_==HOME_FAULT&&possible_&&elapsed(p.nowUs(),lastOffUs_)>=100000)writeOff(p);
    if(phase_!=HOME_RELEASING&&phase_!=HOME_FAULT&&(!currentStarted_||elapsed(p.nowUs(),lastCurrentUs_)>=100000)){
      currentStarted_=true;lastCurrentUs_=p.nowUs();startQuery(p,Q_CURRENT);return;
    }
    Query q=Q_POS;
    if(phase_==HOME_PREPARING)q=stage_==0?Q_MODE:stage_==1?Q_TORQUE:Q_POS;
    else if(phase_==HOME_ENABLING&&stage_<5)q=stage_==0?Q_MODE:stage_==1?Q_TORQUE:stage_==2?Q_POS:stage_==3?Q_GOAL:Q_TORQUE;
    else if(phase_==HOME_RELEASING||phase_==HOME_FAULT)q=stage_==0?Q_MODE:Q_TORQUE;
    else if(phase_==HOME_PREPARED){q=cycle_==0?Q_MODE:cycle_==1?Q_TORQUE:sequential_&&cycle_==2?Q_GOAL:Q_POS;cycle_=(cycle_+1)%(sequential_?4:3);}
    else {q=cycle_==0?Q_MODE:cycle_==1?Q_TORQUE:cycle_==2?Q_GOAL:Q_POS;cycle_=(cycle_+1)%4;}
    startQuery(p,q);
  }
  template<class Port>void step(Port& p){
    checkDeadlines(p);unsigned budget=256;
    while(budget--&&p.available()){int v=p.read();if(v<0)break;if(queryActive_&&!expired(p.nowUs(),queryDeadline_))parse((uint8_t)v,p.nowUs());}
    if(queryActive_&&(received_==kAllMask||expired(p.nowUs(),queryDeadline_))){bool all=received_==kAllMask;queryActive_=false;rxState_=used_=0;finish(p,all);}
  }
  void snapshot(HomeSnapshot& out,uint32_t now,uint32_t seq)const{
    memset(&out,0,sizeof(out));memcpy(out.magic,"DHS1",4);out.version=HOME_CONTRACT_VERSION;
    if(sequential_){out.bootNonce=boot_;out.stateSeq=seq;out.sampleUs=now;out.result=OK;return;}
    out.phase=phase_;out.result=result_;out.error=error_;out.bootNonce=boot_;out.stateSeq=seq;out.sampleUs=now;
    out.operationAckSeq=ack_;out.session=nonce_;out.expiresUs=reportedDeadline();memcpy(out.nativeSha256,sha_,32);
    out.targetMask=phase_==HOME_OFF?0:kAllMask;out.freshMask=freshPositionMask(now);out.steps=steps_;
    // arrivedMask is a latched completion prefix, guarded continuously thereafter.
    out.arrivedMask=completedMask_;out.activeId=error_?faultId_:activeId();out.stage=error_?faultStage_:publicStage();
    for(unsigned i=0;i<kCount;++i){const Cell& c=cells_[i];uint16_t bit=1u<<i;
      out.targets[i]=targets_[i];out.goals[i]=goals_[i];out.positions[i]=posFresh(c,now)?c.position:kUnknownPosition;
      out.modes[i]=c.modeSeen?c.mode:255;out.torques[i]=c.torqueSeen?c.torque:255;
      if((commandedMask_&bit)&&goalFresh(c,now)&&c.goal==word(goals_[i]))out.primedMask|=bit;
      if(metaFresh(c,now)&&c.torque==1)out.enabledMask|=bit;
      if(metaFresh(c,now)&&c.torque==0)out.offMask|=bit;
    }
    if(phase_==HOME_DONE||phase_==HOME_FAULT)out.offMask=offMask_;
  }
  void sequenceSnapshot(SequenceSnapshot& out,uint32_t now,uint32_t seq)const{
    memset(&out,0,sizeof(out));memcpy(out.magic,"DZS1",4);out.version=1;
    out.bootNonce=boot_;out.stateSeq=seq;out.sampleUs=now;
    if(!sequential_){out.result=OK;return;}
    out.phase=phase_==HOME_ENABLING?(captureUs_?4:3):phase_;out.result=result_;out.error=error_;
    out.operationAckSeq=ack_;out.session=nonce_;out.expiresUs=nonce_?lastBeat_+kHeartbeatUs:0;out.lastRequestSeq=lastRequest_;
    out.selectionSeq=selectionSeq_;out.captureSeq=captureSeq_;out.heldMask=completedMask_;out.freshMask=freshPositionMask(now);
    out.selectedId=selected_?kIds[active_]:0;out.stage=error_?faultStage_:publicStage();
    out.capturePosition=capturePosition_;out.captureMode=captureMode_;out.captureStartUs=captureStartUs_;out.captureUs=captureUs_;out.heldUs=heldUs_;
    for(unsigned index=0;index<kCount;++index){const Cell& cell=cells_[index];const uint16_t bit=1u<<index;
      out.goals[index]=goals_[index];out.positions[index]=posFresh(cell,now)?cell.position:kUnknownPosition;
      out.modes[index]=cell.modeSeen?cell.mode:255;out.torques[index]=cell.torqueSeen?cell.torque:255;
      if(metaFresh(cell,now)){out.metadataMask|=bit;if(cell.torque==0)out.offMask|=bit;else out.enabledMask|=bit;}
    }
    if(phase_==HOME_DONE||phase_==HOME_FAULT)out.offMask=offMask_;
  }
  void currentSnapshot(CurrentSnapshot& out,uint32_t now,uint32_t seq)const{
    memset(&out,0,sizeof(out));memcpy(out.magic,"DIC1",4);out.version=1;out.bootNonce=boot_;out.stateSeq=seq;out.sampleUs=now;
    for(unsigned index=0;index<kCount;++index){const Cell& cell=cells_[index];out.rawWords[index]=cell.currentRaw;out.agesMs[index]=cell.currentSeen?age(now,cell.currentUs):UINT16_MAX;if(cell.currentSeen&&elapsed(now,cell.currentUs)<=kSourceMaxUs)out.validMask|=1u<<index;}
  }
  void calibrationSnapshot(Snapshot& out,uint32_t now,uint32_t seq)const{
    memset(&out,0,sizeof(out));memcpy(out.magic,"DWR1",4);out.version=SNAPSHOT_VERSION;
    out.bootNonce=boot_;out.stateSeq=seq;out.sampleUs=now;out.operationAckSeq=ack_;out.lastReceivedReqSeq=lastRequest_;
    out.session=nonce_;out.expiresUs=reportedDeadline();out.flags=POLICY_LOCKED|BOOT_READY;
    if(nonce_)out.flags|=SESSION_ACTIVE;if(possible_)out.flags|=TORQUE_MAY_BE_ON;if(ever_)out.flags|=TORQUE_ENABLE_EVER_ATTEMPTED;
    if(!possible_&&actualOff(now))out.flags|=ALL_TORQUE_OFF_VERIFIED;
    out.state=phase_==HOME_RELEASING?VERIFY_OFF:phase_==HOME_FAULT&&possible_?FAULT:phase_==HOME_MOVING||phase_==HOME_HOLDING?MOVING:phase_==HOME_PREPARING||phase_==HOME_PREPARED||phase_==HOME_ENABLING?PREFLIGHT:IDLE;
    out.result=PENDING;out.op=READ;out.anchor=out.goal=kUnknownPosition;out.diagMin=out.diagMax=UINT16_MAX;
    for(unsigned i=0;i<kCount;++i){const Cell& c=cells_[i];Row& r=out.rows[i];r.rawWord=c.raw;r.signedSteps=c.posSeen?c.position:kUnknownPosition;
      r.positionAgeMs=c.posSeen?age(now,c.posUs):UINT16_MAX;
      r.metadataAgeMs=c.modeSeen&&c.torqueSeen?age(now,elapsed(now,c.modeUs)>elapsed(now,c.torqueUs)?c.modeUs:c.torqueUs):UINT16_MAX;
      r.mode=c.modeSeen?c.mode:255;r.torque=c.torqueSeen?c.torque:255;r.voltageRaw=c.voltage;r.temperature=c.temperature;
      if(posFresh(c,now))out.positionMask|=1u<<i;if(metaFresh(c,now))out.metadataMask|=1u<<i;
    }
  }
 private:
  enum Query:uint8_t{Q_MODE,Q_TORQUE,Q_POS,Q_GOAL,Q_CURRENT};
  struct Cell{uint16_t raw=0,goal=0,currentRaw=0;int16_t position=0,speed=0;uint8_t mode=255,torque=255,voltage=0,temperature=0;bool posSeen=false,modeSeen=false,torqueSeen=false,goalSeen=false,currentSeen=false;uint32_t posUs=0,modeUs=0,torqueUs=0,goalUs=0,goalGeneration=0,currentUs=0;};
  Cell cells_[kCount];int16_t targets_[kCount]={},goals_[kCount]={},prime_[kCount]={};uint8_t modes_[kCount]={},sha_[32]={};
  uint32_t boot_=0,nonce_=0,deadline_=0,lastBeat_=0,lastRequest_=0,ack_=0,generation_=0;
  uint32_t jointStartUs_=0,jointBudgetUs_=kHomeJointUs,stableSince_=0,trajectoryUs_=0,velocityMilli_=0,travelMilli_=0,trackSince_[kCount]={};
  uint32_t releaseUs_=0,lastOffUs_=0,queryDeadline_=0;
  uint32_t selectionSeq_=0,captureSeq_=0,captureStartUs_=0,captureUs_=0,heldUs_=0;
  int16_t expectedPosition_=0,capturePosition_=kUnknownPosition,captureMinimum_=0,captureMaximum_=0;
  uint8_t captureMode_=255;
  uint8_t phase_=HOME_OFF,result_=OK,error_=NONE,active_=0,stage_=0,cycle_=0,faultId_=0,faultStage_=0,pendingFaultId_=0;
  uint8_t rxState_=0,used_=0,length_=0,rx_[16]={};uint16_t received_=0,tickMask_=0,offMask_=0,steps_=0,commandedMask_=0,completedMask_=0;
  bool initialized_=false,possible_=false,ever_=false,queryActive_=false;Query query_=Q_POS;
  bool sequential_=false,selected_=false,sequencePolling_=false;
  uint32_t lastCurrentUs_=0;bool currentStarted_=false;
  static uint16_t word(int16_t p){return p<0?(uint16_t)(0x8000u|(uint16_t)-(int32_t)p):(uint16_t)p;}
  static uint16_t age(uint32_t now,uint32_t t){uint32_t v=elapsed(now,t)/1000;return v>=UINT16_MAX?UINT16_MAX:(uint16_t)v;}
  static bool posFresh(const Cell& c,uint32_t now){return c.posSeen&&elapsed(now,c.posUs)<=kSourceMaxUs;}
  static bool metaFresh(const Cell& c,uint32_t now){return c.modeSeen&&c.torqueSeen&&c.mode!=255&&c.torque<=1&&elapsed(now,c.modeUs)<=kSourceMaxUs&&elapsed(now,c.torqueUs)<=kSourceMaxUs;}
  bool goalFresh(const Cell& c,uint32_t now)const{return c.goalSeen&&c.goalGeneration==generation_&&elapsed(now,c.goalUs)<=kSourceMaxUs;}
  bool actualOff(uint32_t now)const{for(unsigned i=0;i<kCount;++i)if(!metaFresh(cells_[i],now)||cells_[i].torque!=0)return false;return true;}
  uint8_t activeId()const{return (phase_==HOME_ENABLING||phase_==HOME_MOVING)&&active_<kCount?kIds[active_]:0;}
  uint32_t reportedDeadline()const{return nonce_?(sequential_||phase_==HOME_HOLDING?lastBeat_+kHeartbeatUs:deadline_):0;}
  uint8_t publicStage()const{
    if(phase_==HOME_PREPARING)return stage_+HS_READ_MODE;
    if(phase_==HOME_ENABLING)return stage_<5?stage_+HS_READ_MODE:HS_PRIME_SETTLE;
    if(phase_==HOME_MOVING)return goals_[active_]==targets_[active_]?HS_FINAL_SETTLE:HS_RAMP;
    if(phase_==HOME_HOLDING)return HS_HOLD;
    if(phase_==HOME_RELEASING||phase_==HOME_FAULT)return HS_RELEASE;
    return HS_IDLE;
  }
  void startJoint(uint32_t now){phase_=HOME_ENABLING;stage_=cycle_=0;jointStartUs_=now;jointBudgetUs_=homeJointBudgetUs(kIds[active_],cells_[active_].position,targets_[active_]);stableSince_=0;trajectoryUs_=now;velocityMilli_=travelMilli_=0;}
  bool withinHomePath(unsigned i)const{
    const int32_t position=cells_[i].position,target=targets_[i],prime=prime_[i];
    if(!homePositionAllowed(kIds[i],cells_[i].position,targets_[i]))return false;
    if(!homeHead(kIds[i]))return magnitude(position-prime)<=1024;
    // Unpowered joints may be repositioned before their fresh prime read.
    // Once a goal has been written, retain a fixed start-to-target corridor;
    // tracking still gates every1-2tick step and faults at33ticks.
    if(!(commandedMask_&(1u<<i)))return true;
    const int32_t lo=prime<target?prime:target,hi=prime>target?prime:target;
    return position>=lo-kHomeHardErrorTicks&&position<=hi+kHomeHardErrorTicks;
  }
  uint8_t guard(uint32_t now,uint16_t expectedOn){
    for(unsigned i=0;i<kCount;++i){const Cell& c=cells_[i];uint8_t e=NONE;
      if(!posFresh(c,now)||!metaFresh(c,now))e=FEEDBACK;
      else if(c.mode!=modes_[i]||(c.mode!=0&&c.mode!=4))e=MODE;
      else if(c.raw==0x8000u||c.position!=signedPosition(c.raw)||c.mode==0&&(c.position<0||c.position>4095))e=POSITION;
      else if(c.torque!=((expectedOn>>i)&1))e=TORQUE_OFF;
      else if(c.voltage<40||c.voltage>84)e=VOLTAGE;
      else if(c.temperature>=65)e=TEMPERATURE;
      else if(!sequential_&&!withinHomePath(i))e=RANGE;
      if(e){pendingFaultId_=kIds[i];return e;}
    }return NONE;
  }
  uint8_t tracking(uint32_t now){
    for(unsigned i=0;i<kCount;++i)if(commandedMask_&(1u<<i)){
      const int32_t error=magnitude((int32_t)cells_[i].position-goals_[i]);
      if(error>kHomeHardErrorTicks){pendingFaultId_=kIds[i];return POSITION;}
      if(error>kHomeTrackTicks){if(!trackSince_[i])trackSince_[i]=now;else if(elapsed(now,trackSince_[i])>=kHomeTrackUs){pendingFaultId_=kIds[i];return MOVE_TIMEOUT;}}
      else trackSince_[i]=0;
    }return NONE;
  }
  bool trackingReady()const{
    for(unsigned i=0;i<kCount;++i)if((commandedMask_&(1u<<i))&&magnitude((int32_t)cells_[i].position-goals_[i])>kHomeTrackTicks)return false;
    return true;
  }
  bool stable(uint32_t now,int16_t goal){
    if(magnitude((int32_t)cells_[active_].position-goal)>kHomeArrivalTicks){stableSince_=0;return false;}
    if(!stableSince_){stableSince_=now;return false;}
    return elapsed(now,stableSince_)>=kHomeStableUs;
  }
  bool allStable(uint32_t now){
    for(unsigned i=0;i<kCount;++i)if(magnitude((int32_t)cells_[i].position-targets_[i])>kHomeArrivalTicks){stableSince_=0;return false;}
    if(!stableSince_){stableSince_=now;return false;}
    return elapsed(now,stableSince_)>=kHomeStableUs;
  }
  template<class Port>void interrupt(Port& p){queryActive_=false;rxState_=used_=0;unsigned n=256;while(n--&&p.available())p.read();}
  // Single-ID sync write avoids unicast status ACKs contaminating read replies.
  // Register42/43 alone: never touch HLS current44 or unverified mode4 speed46.
  template<class Port>bool writeGoal(Port& p){
    ever_=possible_=true;commandedMask_|=1u<<active_;
    const uint16_t v=word(goals_[active_]);uint8_t b[11]={255,255,254,7,0x83,42,2,kIds[active_],(uint8_t)v,(uint8_t)(v>>8),0};
    cells_[active_].goalSeen=false;uint8_t sum=0;for(unsigned i=2;i<10;++i)sum=(uint8_t)(sum+b[i]);b[10]=(uint8_t)~sum;
    const bool ok=p.write(b,sizeof(b))==sizeof(b);p.flush();return ok;
  }
  template<class Port>bool enable(Port& p){
    ever_=possible_=true;uint8_t b[10]={255,255,254,6,0x83,40,1,kIds[active_],1,0};
    cells_[active_].torqueSeen=false;uint8_t sum=0;for(unsigned i=2;i<9;++i)sum=(uint8_t)(sum+b[i]);b[9]=(uint8_t)~sum;
    const bool ok=p.write(b,sizeof(b))==sizeof(b);p.flush();return ok;
  }
  template<class Port>void writeOff(Port& p){
    lastOffUs_=p.nowUs();uint8_t b[8]={255,255,254,4,3,40,0,0};uint8_t sum=0;for(unsigned i=2;i<7;++i)sum=(uint8_t)(sum+b[i]);b[7]=(uint8_t)~sum;
    for(unsigned i=0;i<kCount;++i)cells_[i].torqueSeen=false;
    if(p.write(b,8)!=8&&!error_){error_=TORQUE_OFF;faultId_=activeId();faultStage_=publicStage();}p.flush();
  }
  template<class Port>void release(Port& p,uint8_t error){
    if(error&&!error_){faultId_=pendingFaultId_?pendingFaultId_:activeId();faultStage_=publicStage();error_=error;}
    interrupt(p);nonce_=deadline_=0;phase_=HOME_RELEASING;result_=PENDING;offMask_=0;stage_=0;releaseUs_=p.nowUs();writeOff(p);
    if(sequential_)completedMask_=0;
  }
  template<class Port>void checkDeadlines(Port& p){
    if(!nonce_)return;const uint32_t now=p.nowUs();
    if(elapsed(now,lastBeat_)>=kHeartbeatUs){release(p,SESSION);return;}
    if((!sequential_&&phase_!=HOME_HOLDING||sequential_&&phase_==HOME_PREPARING)&&expired(now,deadline_)){release(p,phase_==HOME_PREPARING||phase_==HOME_PREPARED?SESSION:MOVE_TIMEOUT);return;}
    if((phase_==HOME_ENABLING||phase_==HOME_MOVING)&&elapsed(now,jointStartUs_)>=jointBudgetUs_)release(p,MOVE_TIMEOUT);
  }
  template<class Port>void startQuery(Port& p,Query q){
    unsigned n=256;while(n--&&p.available())p.read();query_=q;queryActive_=true;received_=0;rxState_=used_=0;
    uint8_t b[23]={255,255,254,19,0x82,(uint8_t)(q==Q_MODE?33:q==Q_TORQUE?40:q==Q_GOAL?42:q==Q_CURRENT?69:56),(uint8_t)(q==Q_POS?8:q==Q_GOAL||q==Q_CURRENT?2:1)};
    for(unsigned i=0;i<kCount;++i)b[7+i]=kIds[i];uint8_t sum=0;for(unsigned i=2;i<22;++i)sum=(uint8_t)(sum+b[i]);b[22]=(uint8_t)~sum;
    queryDeadline_=p.nowUs()+kReadWindowUs;if(p.write(b,23)!=23)queryDeadline_=p.nowUs();p.flush();
  }
  void parse(uint8_t v,uint32_t now){
    if(rxState_==0){if(v==255){rx_[0]=v;rxState_=1;}return;}
    if(rxState_==1){rxState_=v==255?2:0;return;}
    if(rxState_==2){if(v==255)return;rx_[2]=v;rxState_=3;return;}
    if(rxState_==3){uint8_t expected=query_==Q_POS?10:query_==Q_GOAL||query_==Q_CURRENT?4:3;if(v!=expected){rxState_=v==255?1:0;return;}rx_[3]=v;length_=v;used_=0;rxState_=4;return;}
    if(4u+used_>=sizeof(rx_)){rxState_=0;return;}rx_[4+used_++]=v;if(used_<length_)return;rxState_=0;
    uint8_t sum=0;for(unsigned i=2;i<3u+length_;++i)sum=(uint8_t)(sum+rx_[i]);int idx=indexFor(rx_[2]);
    if(idx<0||rx_[4]||(uint8_t)~sum!=rx_[3+length_]||(received_&(1u<<idx)))return;
    received_|=1u<<idx;Cell& c=cells_[idx];
    if(query_==Q_CURRENT){c.currentRaw=(uint16_t)rx_[5]|((uint16_t)rx_[6]<<8);c.currentSeen=true;c.currentUs=now;}
    else if(query_==Q_MODE){c.mode=rx_[5];c.modeSeen=true;c.modeUs=now;}
    else if(query_==Q_TORQUE){c.torque=rx_[5];c.torqueSeen=true;c.torqueUs=now;}
    else if(query_==Q_GOAL){c.goal=(uint16_t)rx_[5]|((uint16_t)rx_[6]<<8);c.goalSeen=true;c.goalUs=now;c.goalGeneration=generation_;}
    else{c.raw=(uint16_t)rx_[5]|((uint16_t)rx_[6]<<8);c.position=signedPosition(c.raw);c.speed=signedPosition((uint16_t)rx_[7]|((uint16_t)rx_[8]<<8));c.voltage=rx_[11];c.temperature=rx_[12];c.posSeen=true;c.posUs=now;tickMask_|=1u<<idx;}
  }
  bool echoed(uint32_t now)const{for(unsigned i=0;i<kCount;++i)if((commandedMask_&(1u<<i))&&(!goalFresh(cells_[i],now)||cells_[i].goal!=word(goals_[i])))return false;return true;}
  template<class Port>void advanceTrajectory(Port& p,uint32_t now){
    uint32_t dt=elapsed(now,trajectoryUs_);trajectoryUs_=now;if(dt>50000)dt=50000;
    if(!trackingReady()){velocityMilli_=travelMilli_=0;return;}
    const uint32_t dv=(uint32_t)((uint64_t)kHomeAccelerationMilli*dt/1000000);
    const uint32_t previousVelocity=velocityMilli_;
    const uint32_t remainingMilli=(uint32_t)magnitude((int32_t)targets_[active_]-goals_[active_])*1000-travelMilli_;
    const uint32_t brakingMilli=(uint32_t)(((uint64_t)velocityMilli_*velocityMilli_+2*kHomeAccelerationMilli-1)/(2*kHomeAccelerationMilli));
    const uint32_t nextTravelMilli=(uint32_t)((uint64_t)velocityMilli_*dt/1000000);
    if(remainingMilli<=brakingMilli+nextTravelMilli)velocityMilli_=velocityMilli_>dv?velocityMilli_-dv:0;
    else velocityMilli_=velocityMilli_+dv>kHomeVelocityMilli?kHomeVelocityMilli:velocityMilli_+dv;
    // Integrate the bounded velocity trapezoid, then quantize to1--2ticks.
    travelMilli_+=(uint32_t)((uint64_t)(previousVelocity+velocityMilli_)*dt/2000000);
    uint32_t step=travelMilli_/1000;travelMilli_%=1000;if(step>2)step=2;if(!step)return;
    int32_t d=(int32_t)targets_[active_]-goals_[active_];if(magnitude(d)<(int32_t)step)step=(uint32_t)magnitude(d);
    if(!step)return;goals_[active_]=(int16_t)(goals_[active_]+(d<0?-(int32_t)step:(int32_t)step));++steps_;
    if(!writeGoal(p)){release(p,FEEDBACK);return;}if(goals_[active_]==targets_[active_])stableSince_=0;
  }
  template<class Port>void finish(Port& p,bool all){
    const uint32_t now=p.nowUs();
    if(query_==Q_CURRENT)return;
    if(phase_==HOME_RELEASING||phase_==HOME_FAULT){
      if(query_==Q_MODE){stage_=1;return;}uint16_t off=0;
      for(unsigned i=0;i<kCount;++i)if(cells_[i].torqueSeen&&elapsed(now,cells_[i].torqueUs)<=kSourceMaxUs&&cells_[i].torque==0)off|=1u<<i;
      if(all&&off==kAllMask){offMask_=off;possible_=false;phase_=error_?HOME_FAULT:HOME_DONE;result_=error_?ERROR:OK;}
      else if(elapsed(now,releaseUs_)>=150000){phase_=HOME_FAULT;result_=ERROR;possible_=true;if(!error_){error_=TORQUE_OFF;faultStage_=HS_RELEASE;for(unsigned i=0;i<kCount;++i)if(!(off&(1u<<i))){faultId_=kIds[i];break;}}}
      return;
    }
    if(!all){for(unsigned i=0;i<kCount;++i)if(!(received_&(1u<<i))){pendingFaultId_=kIds[i];break;}release(p,INCOMPLETE);return;}
    if(phase_==HOME_PREPARING){
      if(stage_<2){++stage_;return;}for(unsigned i=0;i<kCount;++i){prime_[i]=goals_[i]=cells_[i].position;if(sequential_)modes_[i]=cells_[i].mode;}
      const uint8_t e=guard(now,0);if(e){release(p,e);return;}offMask_=kAllMask;phase_=HOME_PREPARED;result_=OK;cycle_=0;return;
    }
    if(phase_==HOME_ENABLING&&stage_<5){
      if(stage_<2){++stage_;return;}
      if(stage_==2){
        const uint8_t e=guard(now,completedMask_);if(e){release(p,e);return;}
        const uint8_t track=tracking(now);if(track){release(p,track);return;}
        // A previous held joint may sag while the next joint is still off.
        // Do not establish a new target or enable it during that recovery window.
        if(!trackingReady()){stage_=0;return;}
        if(sequential_){
          const Cell& cell=cells_[active_];
          if(magnitude((int32_t)cell.position-expectedPosition_)>kHomeArrivalTicks){release(p,UNSTABLE);return;}
          if(!captureStartUs_){captureStartUs_=cell.posUs;captureMinimum_=captureMaximum_=cell.position;}
          if(cell.position<captureMinimum_)captureMinimum_=cell.position;if(cell.position>captureMaximum_)captureMaximum_=cell.position;
          if(magnitude((int32_t)captureMaximum_-captureMinimum_)>kHomeArrivalTicks){captureStartUs_=cell.posUs;captureMinimum_=captureMaximum_=cell.position;}
          if(elapsed(cell.posUs,captureStartUs_)<500000){sequencePolling_=true;stage_=3;return;}
          capturePosition_=cell.position;captureMode_=cell.mode;captureUs_=cell.posUs;targets_[active_]=cell.position;
        }
        // Previous joints remain held; only this still-unpowered joint is rebased.
        prime_[active_]=goals_[active_]=cells_[active_].position;trackSince_[active_]=0;
        if(!sequential_)jointBudgetUs_=homeJointBudgetUs(kIds[active_],prime_[active_],targets_[active_]);
        cells_[active_].torqueSeen=false;offMask_=0;
        if(!writeGoal(p)){release(p,FEEDBACK);return;}stage_=3;return;
      }
      if(stage_==3){if(!echoed(now)){release(p,DIAG_ERROR);return;}if(sequencePolling_){sequencePolling_=false;stage_=0;return;}if(!enable(p)){release(p,FEEDBACK);return;}stage_=4;return;}
      for(unsigned i=0;i<kCount;++i)if(cells_[i].torque!=((commandedMask_>>i)&1)){pendingFaultId_=kIds[i];release(p,TORQUE_OFF);return;}
      stage_=5;cycle_=0;stableSince_=0;return;
    }
    if(query_!=Q_POS)return;
    const uint8_t e=guard(now,commandedMask_);if(e){release(p,e);return;}
    if(phase_==HOME_PREPARED&&!sequential_)return;
    if(!echoed(now)){release(p,DIAG_ERROR);return;}
    const uint8_t track=tracking(now);if(track){release(p,track);return;}
    if(phase_==HOME_PREPARED)return;
    if(phase_==HOME_HOLDING)return;
    if(phase_==HOME_ENABLING){
      if(stable(now,prime_[active_])){
        if(sequential_){completedMask_|=1u<<active_;heldUs_=cells_[active_].posUs;phase_=HOME_HOLDING;result_=OK;return;}
        phase_=HOME_MOVING;result_=OK;stableSince_=0;trajectoryUs_=now;velocityMilli_=travelMilli_=0;
      }return;
    }
    if(goals_[active_]==targets_[active_]){
      if(active_==kCount-1){
        // Prior per-joint arrival is historical evidence. All fifteen must be
        // within the final tolerance throughout the same300ms window for HOLD.
        if(!allStable(now))return;
        completedMask_=kAllMask;phase_=HOME_HOLDING;deadline_=0;result_=OK;return;
      }
      if(!stable(now,targets_[active_]))return;
      completedMask_|=1u<<active_;
      ++active_;startJoint(now);return;
    }
    advanceTrajectory(p,now);
  }
};

}
