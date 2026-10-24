"""Six-point single-actor keyboard admission; physical acceptance is pending."""
import math

PROFILE = {'schema':'xgoduck-basic-keyboard-publication-v2',
 'model_sha256':'57cdebf36e7fa23116270dec96f013a6a64511f37311d322443f5297bdc3d6db',
 'ordinary_head_mouth':False, 'walking':True, 'fast_forward':True, 'navigation':True,
 'deep_pick':False, 'automatic_pick':False, 'roulade':False,
 'current_monitor':True, 'calibration_and_numbering':True,
 'single_actor':True, 'native_basic_offline_qualified':False,
 'portable_profile_hardware_verified':False, 'physical_acceptance':'pending',
 'recovery_source':'official walk/getup; direct getup retained; bounded standing observed',
 'head_mouth_scope':'zero only; mixed motion not qualified',
 'command_points':[[0.,0.,0.],[.2,0.,0.],[.3,0.,0.],[-.3,0.,0.],[0.,0.,1.],[0.,0.,-1.]]}

def validate_release_action(action):
    if action in ('pick','roulade'):
        raise ValueError('新版仅验基础行走/停止；深低头、Pick和翻滚尚未验收。')

def _vector(value, length):
    if not isinstance(value,(list,tuple)) or len(value)!=length:
        raise ValueError('command vector length mismatch')
    if any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) for v in value):
        raise ValueError('command must contain finite numbers')
    return [float(v) for v in value]

def constrain_command(body):
    if not isinstance(body,dict):raise ValueError('Expected command object')
    result=dict(body);twist=_vector(result.get('twist',[0.,0.,0.]),3)
    if not any(all(abs(a-b)<1e-7 for a,b in zip(twist,p)) for p in PROFILE['command_points']):
        raise ValueError('只开放W .2、ShiftW .3、S -.3、左右转±1及松键零速；组合运动未验收。')
    head=_vector(result.get('head',[0.]*4),4)
    if any(abs(v)>1e-9 for v in head):raise ValueError('新版基础模型仅接收零头部指令，头身混合尚未验收。')
    for key in ('mouth','manual_pick_depth'):
        value=result.get(key,0.)
        if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or value!=0:
            raise ValueError('新版基础测试固定头嘴/叼取指令为零。')
    for key in ('forward_specialist','navigation_specialist','fast_forward'):
        if key in result and type(result[key]) is not bool:raise ValueError('motion flags must be boolean')
        result[key]=False
    result.update(twist=twist,head=[0.]*4,mouth=0.,manual_pick_depth=0.)
    return result
