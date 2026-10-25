"""Passive MCU receiver evidence, independent of motion admission."""
import struct
import threading
import time

IDS=(10,11,12,13,14,20,21,22,23,24,30,31,32,33,34)
PACKET=struct.Struct('<4s5I15I15I15BH32sB16sB')
class ServoHealth:
    def __init__(self):
        self.value={};self.received=0.;self.lock=threading.Lock();self.invalid=0;self.last_size=None
    def receive(self,raw):
        raw=bytes(raw)
        if len(raw)!=PACKET.size+24 or raw[:4]!=b'DQH2':
            with self.lock:self.invalid+=1;self.last_size=len(raw)
            return
        v=PACKET.unpack_from(raw)
        ticks,rx,crc,length,drained=v[1:6]
        value={'build':raw[PACKET.size:].split(b'\0',1)[0].decode('ascii','replace'),
          'ticks':ticks,'bytes':rx,'checksum_errors':crc,'length_errors':length,'drained_bytes':drained,
          'ids':{str(sid):{'valid_replies':f,'fault_replies':n,'last_error_byte':e} for sid,f,n,e in zip(IDS,v[6:21],v[21:36],v[36:51])},
          'last_incomplete_mask':v[51],'last_incomplete_head':v[52][:min(v[53],32)].hex(),
          'last_bad_frame':v[54][:min(v[55],16)].hex()}
        with self.lock:self.value=value;self.received=time.monotonic()
    def status(self):
        with self.lock:return dict(self.value,age_s=round(time.monotonic()-self.received,3) if self.received else None,
                                   invalid_packets=self.invalid,last_invalid_size=self.last_size)
