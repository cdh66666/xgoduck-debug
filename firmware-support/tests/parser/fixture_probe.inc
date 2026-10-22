static Snapshot run(const Bytes& bytes,const std::vector<size_t>& chunks,int mappedId,int length,bool pending=false) {
  resetServoParser();
  servoRxMask=servoPdMask=0;
  memset(servoRxBuffer,0,sizeof(servoRxBuffer));
  memset(servoBootP,0,sizeof(servoBootP));
  memset(servoBootD,0,sizeof(servoBootD));
  servoPingId=servoReadId=(uint8_t)mappedId;
  servoPingSeen=servoReadSeen=false; servoReadPos=0;
  servoBootPdReading=length==4 && !pending;
  currentSampler.pending=pending; currentSampler.calls.clear(); displayVoltage.calls.clear();
  Serial1.bytes.clear();
  const uint8_t ids[15]={10,11,12,13,14,20,21,22,23,24,30,31,32,33,34};
  for (int i=0;i<15;i++) motors[i]={mappedId>=0?(uint8_t)(i==0?mappedId:254):ids[i],2048,0,1.0f,0.0f,0.0f};
  size_t offset=0;
  for (size_t count:chunks) {
    if (offset+count>bytes.size()) throw std::runtime_error("bad chunk fixture");
    for(size_t i=0;i<count;i++) Serial1.bytes.push_back(bytes[offset++]);
    parseServoRx();
  }
  if(offset!=bytes.size()) throw std::runtime_error("incomplete chunk fixture");
  Snapshot s;
  s.mask=servoRxMask; s.pdmask=servoPdMask; s.ping=servoPingSeen; s.read=servoReadSeen; s.pos=servoReadPos;
  s.flag=servoParserFlag; s.len=servoRxLen; s.dataLen=servoRxDataLen;
  s.voltage=displayVoltage.calls; s.current=currentSampler.calls;
  for(int i=0;i<15;i++) { s.motor[i]={motors[i].fbPos,motors[i].fbAngleDeg,motors[i].fbVelDeg}; s.pd[i]={servoBootP[i],servoBootD[i]}; }
  return s;
}
