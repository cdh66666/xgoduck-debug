"""Training-only bounded direct precision cost, root-confirmed formula."""
import math

PRECISION_SCALES=(.05,.05,.2)
PRECISION_WEIGHT=4.
PRECISION_COST_DIVISOR=2.
PRECISION_GRACE_TICKS=100 #50Hz, UPDATED current tick:100 is exactly2.00s


def direct_precision(command,actual,updated_command_age_ticks):
    moving=math.sqrt(sum(float(value)**2 for value in command))>.04
    active=moving and updated_command_age_ticks>=PRECISION_GRACE_TICKS
    if not active:
        return 0.,dict(active=False,per_second=0.,cost=None,
            command_age_ticks=updated_command_age_ticks,training_only=True)
    normalized=[(float(velocity)-float(target))/scale for velocity,target,scale in zip(actual,command,PRECISION_SCALES)]
    cost=math.fsum(.5*error*error if abs(error)<1. else abs(error)-.5 for error in normalized)
    term=-PRECISION_WEIGHT*math.tanh(cost/PRECISION_COST_DIVISOR)
    return term,dict(active=True,per_second=term,cost=cost,normalized_error=normalized,
        command_age_ticks=updated_command_age_ticks,training_only=True)
