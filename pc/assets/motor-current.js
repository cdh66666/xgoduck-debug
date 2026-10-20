export const CURRENT_IDS=Object.freeze([10,11,12,13,14,20,21,22,23,24,30,31,32,33,34]);
export const CURRENT_MAX_AGE_MS=500;
export class MotorCurrentTracker{
  constructor(){this.samples=new Map();this.held=new Set();this.receivedAt=0;this.boot=null;this.paused=true;}
  consume(message,now,wallNow=Date.now()){
    if(message?.type==='official-servo-current-state'&&message.version===1)return this.consumeOfficial(message,now,wallNow);
    if(message?.type!=='motor-current-state'||message.version!==1)return null;
    if(message.mode!=='live'){this.samples.clear();this.held.clear();this.paused=true;return this.read(now);}
    const transit=wallNow-message.observedAtMs;
    if(!Number.isInteger(message.boot_id)||message.boot_id<=0||!Number.isFinite(message.observedAtMs)||
      !Number.isFinite(transit)||transit<0||!Array.isArray(message.rows)||message.rows.length!==15||
      new Set(message.rows.map(row=>row.id)).size!==15||message.rows.some(row=>!CURRENT_IDS.includes(row.id)))return null;
    this.samples.clear();this.held.clear();this.boot=message.boot_id;this.receivedAt=now;this.paused=false;
    for(const row of message.rows){
      if(row.holding_verified===true&&transit<150)this.held.add(row.id);
      const age=row.current_age_ms+transit;
      if(row.current_source==='servo_phase_current/register69'&&row.current_unit==='mA'&&row.current_fresh===true&&Number.isInteger(row.current_raw)&&row.current_raw>=0&&row.current_raw<=65535&&
        Number.isFinite(row.current_ma)&&Number.isFinite(row.current_age_ms)&&row.current_age_ms>=0&&age<=CURRENT_MAX_AGE_MS){
        this.samples.set(row.id,{ma:row.current_ma,raw:row.current_raw,age});
      }
    }
    return this.read(now);
  }
  consumeOfficial(message,now,wallNow){
    if(message.mode!=='live'){this.samples.clear();this.held.clear();this.paused=true;return this.read(now);}
    const c=message.current,transit=wallNow-message.observedAtMs;
    if(!Number.isFinite(transit)||transit<0||c?.source!=='servo_phase_current'||c.units!=='A'||c.supported!==true||
      !Number.isInteger(c.boot)||c.boot<=0||!Number.isInteger(c.mask)||c.mask<0||c.mask>32767||
      !Number.isFinite(c.age_ms)||c.age_ms<0||!Array.isArray(c.ids)||c.ids.length!==15||
      c.ids.some((id,i)=>id!==CURRENT_IDS[i])||!Array.isArray(c.per_servo_signed_a)||c.per_servo_signed_a.length!==15)return null;
    this.samples.clear();this.held.clear();this.boot=c.boot;this.receivedAt=now;this.paused=false;
    const age=c.age_ms+transit;
    c.per_servo_signed_a.forEach((amps,i)=>{
      if(c.available===true&&(c.mask&(1<<i))&&Number.isFinite(amps)&&age<=CURRENT_MAX_AGE_MS){
        this.samples.set(CURRENT_IDS[i],{ma:amps*1000,raw:null,age});
      }
    });
    return this.read(now);
  }
  read(now){
    const elapsed=now-this.receivedAt;
    return CURRENT_IDS.map(id=>{
      const sample=this.samples.get(id),fresh=!this.paused&&sample&&elapsed>=0&&sample.age+elapsed<=CURRENT_MAX_AGE_MS;
      return {id,fresh:!!fresh,ma:fresh?sample.ma:null,raw:fresh?sample.raw:null,held:!this.paused&&elapsed>=0&&elapsed<150&&this.held.has(id)};
    });
  }
}
export function currentHeat(row,scaleMa=2000){
  return row?.fresh===true&&Number.isFinite(row.ma)?Math.sqrt(Math.min(1,Math.abs(row.ma)/scaleMa)):0;
}
