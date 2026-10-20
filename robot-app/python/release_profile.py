"""Published feature gate, separate from unchanged model/PD/Recovery math."""
import math
PROFILE = {'schema': 'xgoduck-stand-getup-publication-profile-v1', 'core': ['original_walk_zero_standing', 'original_getup'], 'ordinary_head_mouth': True, 'walking': False, 'fast_forward': False, 'navigation': False, 'deep_pick': False, 'automatic_pick': False, 'roulade': False, 'current_monitor': True, 'calibration_and_numbering': True, 'roll_power_loss_cause': 'Unresolved battery power loss; no hardware root-cause fix claimed', 'portable_profile_hardware_verified': False}
def validate_release_action(action):
    if action in ('pick','roulade'):
        raise ValueError('本次发布仅启用原版零速站立/起身；Pick和翻滚入口保留但默认停用。翻滚断电原因未修复。')
def constrain_command(body):
    if not isinstance(body,dict):raise ValueError('Expected command object')
    result=dict(body)
    twist=result.get('twist')
    if twist is not None:
        if len(twist)!=3 or any(not math.isfinite(float(x)) or abs(float(x))>1e-9 for x in twist):
            raise ValueError('发布配置默认关闭移步/行走/快跑；站立/起身只接收零速度。')
    for key in ('forward_specialist','navigation_specialist','fast_forward'):
        if key in result and result[key] is not False:raise ValueError('专用运动策略默认停用')
        result[key]=False
    depth=result.get('manual_pick_depth')
    if depth is not None and (not math.isfinite(float(depth)) or abs(float(depth))>1e-9):
        raise ValueError('深度低头/Pick默认停用；普通头嘴原接口保留。')
    result['twist']=[0.,0.,0.];result['manual_pick_depth']=0.
    return result
