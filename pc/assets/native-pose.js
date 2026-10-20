/* Read-only encoder pose. Official nominal mapping is not installed-axis acceptance. */
export const MOTOR_IDS=Object.freeze([10,11,12,13,14,20,21,22,23,24,30,31,32,33,34]);
export const LINK_BY_ID=Object.freeze({10:'left_hip_yaw',11:'left_hip_roll',12:'left_hip_pitch',13:'left_knee',14:'left_ankle',
  20:'right_hip_yaw',21:'right_hip_roll',22:'right_hip_pitch',23:'right_knee',24:'right_ankle',
  30:'neck_pitch',31:'head_pitch',32:'head_yaw',33:'head_roll',34:'mouth_jaw'});
export const OFFICIAL_SIGN=-1;
export const OFFICIAL_TICKS_PER_TURN=4095; // Official sketch/duck_config.h M_N, not synthetic4096.
export const MAX_POSITION_AGE_MS=150;
const PARENT_ID={10:null,11:10,12:11,13:12,14:13,20:null,21:20,22:21,23:22,24:23,30:null,31:30,32:31,33:32,34:33};
const uint32=value=>Number.isInteger(value)&&value>=0&&value<=0xffffffff;
const array15=value=>Array.isArray(value)&&value.length===15;
const same=(a,b)=>a.length===b.length&&a.every((value,index)=>value===b[index]);
const native=(value,mode)=>Number.isInteger(value)&&(mode===0?value>=0&&value<=4095:mode===4&&value>=-32767&&value<=32767);
const clone=value=>value.map(row=>row.slice());
const identity=()=>[[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]];
export const multiply=(a,b)=>a.map(row=>b[0].map((_,j)=>row.reduce((sum,value,k)=>sum+value*b[k][j],0)));

function rigid(matrix){
  if(!Array.isArray(matrix)||matrix.length!==4||matrix.some(row=>!Array.isArray(row)||row.length!==4||!row.every(Number.isFinite)))return false;
  if(matrix[3].some((value,i)=>Math.abs(value-(i===3?1:0))>1e-8))return false;
  for(let i=0;i<3;i++)for(let j=0;j<3;j++)if(Math.abs(matrix.reduce((sum,row,k)=>k<3?sum+row[i]*row[j]:sum,0)-(i===j?1:0))>1e-6)return false;
  const [a,b,c]=matrix;
  return Math.abs(a[0]*(b[1]*c[2]-b[2]*c[1])-a[1]*(b[0]*c[2]-b[2]*c[0])+a[2]*(b[0]*c[1]-b[1]*c[0])-1)<1e-6;
}
export function inverseRigid(matrix){
  if(!rigid(matrix))throw Error('Invalid rigid model matrix');
  const result=identity();
  for(let i=0;i<3;i++){
    for(let j=0;j<3;j++)result[i][j]=matrix[j][i];
    result[i][3]=-result[i].slice(0,3).reduce((sum,value,j)=>sum+value*matrix[j][3],0);
  }
  return result;
}
const vector3=value=>Array.isArray(value)&&value.length===3&&value.every(Number.isFinite);
function rotation(axis,angle,pivot){
  const [x,y,z]=axis,c=Math.cos(angle),s=Math.sin(angle),t=1-c;
  const result=[[t*x*x+c,t*x*y-s*z,t*x*z+s*y,0],[t*x*y+s*z,t*y*y+c,t*y*z-s*x,0],
    [t*x*z-s*y,t*y*z+s*x,t*z*z+c,0],[0,0,0,1]];
  for(let i=0;i<3;i++)result[i][3]=pivot[i]-result[i].slice(0,3).reduce((sum,value,j)=>sum+value*pivot[j],0);
  return result;
}

export function buildNativeKinematics(data){
  const records=data?.records,zero=data?.poses?.['q-zero']?.records,joints=data?.poses?.['q-zero']?.joints;
  const hasMouth=Array.isArray(records)&&records.some(row=>row.name==='mouth_jaw'),count=hasMouth?16:15;
  if(!Array.isArray(records)||records.length!==count||!zero||!joints||!Array.isArray(data.joints))throw Error('Incomplete q-zero model');
  const known=new Set(['trunk_base',...Object.values(LINK_BY_ID).filter(name=>hasMouth||name!=='mouth_jaw')]);
  if(new Set(records.map(row=>row.name)).size!==count||records.some(row=>!known.has(row.name)||!rigid(zero[row.name])))throw Error('Unexpected model links');
  const entries=new Map();
  for(const record of records){
    if(record.name==='trunk_base'){
      if(record.parent!==null)throw Error('Invalid trunk parent');
      entries.set(record.name,{parent:null,base:clone(zero[record.name])});continue;
    }
    if(!known.has(record.parent)||record.parent===record.name)throw Error('Invalid model parent');
    const id=Number(Object.keys(LINK_BY_ID).find(key=>LINK_BY_ID[key]===record.name));
    const label=data.joints.find(row=>row.id===id),joint=joints[String(id)];
    if(label?.name!==record.name||!joint||!vector3(joint.axis)||!vector3(joint.origin_m)||joint.angle_deg!==0)throw Error('Invalid q-zero joint metadata');
    const inv=inverseRigid(zero[record.name]),length=Math.hypot(...joint.axis);
    if(length<1e-8)throw Error('Missing model axis');
    const axis=inv.slice(0,3).map(row=>row.slice(0,3).reduce((sum,value,j)=>sum+value*joint.axis[j]/length,0));
    const pivot=inv.slice(0,3).map(row=>row[3]+row.slice(0,3).reduce((sum,value,j)=>sum+value*joint.origin_m[j],0));
    entries.set(record.name,{parent:record.parent,index:MOTOR_IDS.indexOf(id),axis,pivot,
      base:multiply(inverseRigid(zero[record.parent]),zero[record.name])});
  }
  // Resolve all ancestors before any mesh write; cycles or missing parents fail as a batch.
  const order=[],visited=new Set(),visiting=new Set();
  function visit(name){
    if(visited.has(name))return;
    if(visiting.has(name))throw Error('Cyclic model tree');
    visiting.add(name);const item=entries.get(name);
    if(!item)throw Error('Missing model parent');
    if(item.parent)visit(item.parent);
    visiting.delete(name);visited.add(name);order.push(name);
  }
  for(const name of entries.keys())visit(name);
  return Object.freeze({version:data.version,order:Object.freeze(order.slice()),mouthGeometryAvailable:hasMouth,
    solve(qRadians){
      if(!array15(qRadians)||!qRadians.every(Number.isFinite))throw Error('Exactly15 finite display angles required');
      const matrices={};
      for(const name of order){
        const item=entries.get(name);
        matrices[name]=item.parent?multiply(multiply(matrices[item.parent],item.base),rotation(item.axis,qRadians[item.index],item.pivot)):clone(item.base);
      }
      return matrices;
    }});
}

export class NativePoseTracker {
  constructor(){this.pose=null;this.mode='waiting';this.reason='等待关节反馈';this.lastNew=null;this.jointTimes=Array(15).fill(null);this.retiredBoots=new Set();this.retiredZeros=new Set();}
  hold(mode,reason){
    this.mode=mode;this.reason=reason;
    if(this.pose)this.pose=Object.freeze({...this.pose,jointLive:Object.freeze(Array(15).fill(false)),encoderLive:Object.freeze(Array(15).fill(false))});
    return {changed:false,sourceChanged:false,pose:this.pose,mode,reason,historical:!!this.pose?.renderKnown.some(Boolean)};
  }
  consume(message,now,wallNow=Date.now()){
    if(message?.type!=='native-pose-state'||message.version!==1)return null;
    if(!['live','stale','paused'].includes(message.mode))return null;
    if(message.mode!=='live'){
      const reason=['等待保存机械零位','等待新鲜关节反馈','页面已暂停'].includes(message.reason)?message.reason:
        message.mode==='paused'?'页面已暂停':'关节反馈离线';
      return this.hold(message.mode,reason);
    }
    const s=message.status;
    if(!Number.isFinite(now)||!Number.isFinite(wallNow)||!s||!uint32(s.boot_id)||!uint32(s.sequence)||
       s.reference_pose!=='q-zero'||typeof s.zero_sha256!=='string'||!(/^[a-f0-9]{64}$/i).test(s.zero_sha256)||
       ![s.positions,s.position_age_ms,s.modes,s.zeros].every(array15)||
       !Number.isFinite(s.age_ms)||s.age_ms<0||!Number.isFinite(message.observedAtMs)||message.observedAtMs<=0)
      return this.hold('stale','关节反馈无效');
    const corrections=s.direction_corrections===undefined?Array(15).fill(1):s.direction_corrections;
    if(!array15(corrections)||corrections.some(value=>value!==1&&value!==-1)||
       s.positions.some(value=>!Number.isInteger(value))||s.modes.some(value=>!Number.isInteger(value))||
       s.zeros.some(value=>!Number.isInteger(value)||value< -32767||value>32767)||
       s.position_age_ms.some(value=>!Number.isFinite(value)||value<0))return this.hold('stale','原生位置或显示方向无效');
    const cachedAge=wallNow-message.observedAtMs;
    const headerAge=s.age_ms+cachedAge;
    if(cachedAge<0||headerAge>=MAX_POSITION_AGE_MS)return this.hold('stale','关节反馈过期');
    const zeroSha=s.zero_sha256.toLowerCase(),old=this.pose;
    if(this.retiredBoots.has(s.boot_id)||this.retiredZeros.has(zeroSha))return this.hold('stale','旧反馈或旧零位版本');
    const sameBoot=old?.bootId===s.boot_id,delta=sameBoot?(s.sequence-old.sequence)>>>0:1;
    if(sameBoot&&delta>=0x80000000)return this.hold('stale','关节反馈倒退');
    if(old?.zeroSha===zeroSha&&!same(old.zeros,s.zeros))return this.hold('stale','零位哈希与内容不一致');
    const sourceChanged=!sameBoot||delta!==0;
    const changedFreshValue=old&&s.positions.some((value,i)=>value!==old.sourcePositions[i]&&
      s.position_age_ms[i]+cachedAge<MAX_POSITION_AGE_MS&&native(value,s.modes[i]));
    if(!sourceChanged&&(changedFreshValue||this.lastNew===null||now-this.lastNew>=MAX_POSITION_AGE_MS))
      return this.hold('stale','重复关节反馈无效或过期');
    const configurationChanged=!old||old.zeroSha!==zeroSha||!same(old.directionCorrections,corrections);
    if(old&&(!sameBoot||old.zeroSha!==zeroSha)&&(this.retiredBoots.size>=256||this.retiredZeros.size>=256))
      return this.hold('stale','姿态版本历史已满，请重新打开页面');
    const newZero=!old||old.zeroSha!==zeroSha;
    const zeroModes=s.modes.map((mode,i)=>!newZero&&old.modes[i]!==null?old.modes[i]:[0,4].includes(mode)?mode:null);
    const encoderLive=s.positions.map((value,i)=>native(value,s.modes[i])&&native(s.zeros[i],s.modes[i])&&s.modes[i]===zeroModes[i]&&
      s.position_age_ms[i]+cachedAge<MAX_POSITION_AGE_MS&&(sourceChanged||this.jointTimes[i]!==null&&now-this.jointTimes[i]<MAX_POSITION_AGE_MS));
    const jointLive=encoderLive.slice();
    // MOTOR_IDS is topological within each of the three independent branches.
    for(const [i,id] of MOTOR_IDS.entries())if(PARENT_ID[id]!==null)jointLive[i]=jointLive[i]&&jointLive[MOTOR_IDS.indexOf(PARENT_ID[id])];
    if(old&&newZero&&jointLive.slice(0,14).some(value=>!value))return this.hold('stale','新零位等待14关节有效反馈');
    const positions=s.positions.map((value,i)=>encoderLive[i]?value:old?.positions[i]??null);
    const rendered=s.positions.map((value,i)=>jointLive[i]?value:old?.renderPositions[i]??null);
    const difference=positions.map((value,i)=>value===null?null:value-s.zeros[i]);
    // A new zero cannot reinterpret a frozen mouth sample as a fresh angle.
    const radians=rendered.map((value,i)=>!jointLive[i]&&old?.renderKnown[i]?old.qRadians[i]:value===null?0:OFFICIAL_SIGN*corrections[i]*(value-s.zeros[i])*2*Math.PI/OFFICIAL_TICKS_PER_TURN);
    const freeze=value=>Object.freeze(value.slice());
    const pose=Object.freeze({ids:MOTOR_IDS,bootId:s.boot_id,sequence:s.sequence,zeroSha,referencePose:'q-zero',
      positions:freeze(positions),renderPositions:freeze(rendered),renderKnown:freeze(rendered.map(value=>value!==null)),
      sourcePositions:freeze(s.positions),sourceModes:freeze(s.modes),zeros:freeze(s.zeros),modes:freeze(zeroModes),directionCorrections:freeze(corrections),
      encoderLive:freeze(encoderLive),jointLive:freeze(jointLive),deltaNative:freeze(difference),qRadians:freeze(radians),
      installationDirectionsVerified:false,physicalAcceptanceVerified:false});
    if(old&&!sameBoot)this.retiredBoots.add(old.bootId);
    if(old&&old.zeroSha!==zeroSha)this.retiredZeros.add(old.zeroSha);
    this.pose=pose;
    if(sourceChanged){this.lastNew=now-headerAge;for(let i=0;i<15;i++)if(encoderLive[i])this.jointTimes[i]=now-s.position_age_ms[i]-cachedAge;}
    this.mode=jointLive.slice(0,14).some(Boolean)?'live':'stale';this.reason='编码器姿态 · 安装方向待核对';
    return {changed:sourceChanged||configurationChanged||!same(old.qRadians,radians),sourceChanged,pose,mode:this.mode,reason:this.reason,historical:jointLive.some(value=>!value)};
  }
  tick(now){
    if(!this.pose||this.mode!=='live')return null;
    if(this.lastNew===null||now-this.lastNew>=MAX_POSITION_AGE_MS)return this.hold('stale','关节反馈过期');
    const encoderLive=this.pose.encoderLive.map((live,i)=>live&&this.jointTimes[i]!==null&&now-this.jointTimes[i]<MAX_POSITION_AGE_MS),jointLive=encoderLive.slice();
    for(const [i,id] of MOTOR_IDS.entries())if(PARENT_ID[id]!==null)jointLive[i]=jointLive[i]&&jointLive[MOTOR_IDS.indexOf(PARENT_ID[id])];
    if(same(jointLive,this.pose.jointLive)&&same(encoderLive,this.pose.encoderLive))return null;
    this.pose=Object.freeze({...this.pose,encoderLive:Object.freeze(encoderLive),jointLive:Object.freeze(jointLive)});
    this.mode=jointLive.slice(0,14).some(Boolean)?'live':'stale';
    return {changed:false,sourceChanged:false,pose:this.pose,mode:this.mode,reason:'部分关节反馈过期',historical:true};
  }
}
