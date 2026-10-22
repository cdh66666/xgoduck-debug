"""Training-only same-command progress window. No simulator or ML imports."""
from collections import deque

HOLD_TICKS=150
WINDOW_TICKS=50
MIN_SIGNED_RATIO=.25
STALL_EVENT_PENALTY=1.


def event_outcome(physical_failure,motion_stall,episode_timeout):
    terminated=bool(physical_failure or motion_stall)
    timeout=bool(episode_timeout and not terminated)
    event_penalty=(1. if physical_failure else 0.)+(STALL_EVENT_PENALTY if motion_stall else 0.)
    return terminated,timeout,event_penalty


class MotionWindow:
    def __init__(self):
        self.samples=deque(maxlen=WINDOW_TICKS)
        self.command=None
        self.segment_id=0

    def reset(self,command):
        # Every resample, including the SAME value, starts a new finite segment.
        self.command=tuple(float(value) for value in command)
        self.samples.clear()
        self.segment_id+=1

    def observe(self,command,velocity,command_age_ticks):
        key=tuple(float(value) for value in command)
        if key!=self.command:
            raise ValueError('motion_window_command_changed_without_resample_reset')
        axes=[index for index in (0,2) if abs(key[index])>.04]
        if not axes:
            return self.summary(command_age_ticks)
        if len(axes)!=1:
            raise ValueError('motion_guard_is_for_frozen_six_single_axis_commands_only')
        axis=axes[0]
        signed=float(velocity[axis])*(1. if key[axis]>0. else -1.)
        self.samples.append(signed) # negative progress is NEVER clamped
        return self.summary(command_age_ticks)

    def summary(self,command_age_ticks):
        key=self.command
        axes=[index for index in (0,2) if abs(key[index])>.04]
        axis=axes[0] if len(axes)==1 else None
        target=abs(key[axis]) if axis is not None else None
        mean=sum(self.samples)/len(self.samples) if self.samples else None
        ratio=mean/target if mean is not None and target else None
        eligible=axis is not None and command_age_ticks>=HOLD_TICKS and len(self.samples)==WINDOW_TICKS
        triggered=bool(eligible and ratio<MIN_SIGNED_RATIO)
        return dict(triggered=triggered,eligible=eligible,segment_id=self.segment_id,
            terminal_command=list(key),command_age_ticks=command_age_ticks,
            axis=axis,target_axis_magnitude=target,window_samples=len(self.samples),
            required_window_samples=WINDOW_TICKS,window_signed_axis_mean=mean,signed_axis_ratio=ratio,
            reason='motion_stall' if triggered else None,zero_command_excluded=axis is None,
            training_only=True,acceptance_pass=False)
