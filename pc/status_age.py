"""Age a received snapshot without refreshing its observation or history."""
import copy
import math


def delivery_age_lower_bound_ms(state):
    """Extra age after native caching, relative to recent best source progress.

    This is not absolute network latency. When the native cache age is absent,
    the native feedback age is a conservative upper bound on that cache time.
    """
    native = state.get('feedback_age_ms')
    progress = state.get('pc_source_progress')
    backlog = progress.get('relative_backlog_lower_bound_s') if isinstance(progress, dict) else None
    valid = lambda value: isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value) and value >= 0
    if not valid(native) or not valid(backlog):
        return 0.
    cache = state.get('telemetry_cache_age_ms')
    return max(0., backlog*1000-(cache if valid(cache) else native))


def aged_snapshot(state, received_at, now):
    result = copy.deepcopy(state)
    cached_ms = max(0., now-received_at)*1000
    result['pc_cache_age_ms'] = cached_ms
    native = state.get('feedback_age_ms')
    if isinstance(native, (int, float)) and not isinstance(native, bool) and math.isfinite(native) and native >= 0:
        delivery_ms = delivery_age_lower_bound_ms(state)
        result['pc_delivery_age_lower_bound_ms'] = delivery_ms
        result['native_feedback_age_ms'] = native
        result['native_full_robot_ready'] = state.get('full_robot_ready')
        result['feedback_age_ms'] = native+delivery_ms+cached_ms
        if result['feedback_age_ms'] >= 150:
            result['full_robot_ready'] = False
            result['control_ready'] = False
    else:
        delivery_ms = 0.
        result['full_robot_ready'] = False
        result['control_ready'] = False
    if isinstance(state.get('joint_feedback_age_ms'), list):
        result['joint_feedback_age_ms'] = [None if age is None else age+delivery_ms+cached_ms for age in state['joint_feedback_age_ms']]
        ids = state.get('motors') or [10,11,12,13,14,20,21,22,23,24,30,31,32,33,34]
        result['stale_joint_ids'] = [sid for sid, age in zip(ids,result['joint_feedback_age_ms']) if age is None or age >= 150]
        if result['stale_joint_ids']:
            result['full_robot_ready'] = result['control_ready'] = False
    return result
