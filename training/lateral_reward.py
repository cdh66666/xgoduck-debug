"""Independent root-vy precision cost; training only, no physical action."""
import math

def lateral_precision(command,actual,updated_command_age_ticks):
    active=math.sqrt(sum(float(v)**2 for v in command))>.04 and updated_command_age_ticks>=100
    if not active:
        return 0.,dict(active=False,per_second=0.,cost=None,command_age_ticks=updated_command_age_ticks,training_only=True)
    z=float(actual[1])/.05
    h=.5*z*z if abs(z)<1. else abs(z)-.5
    rate=-2.*h/(1.+h)
    return rate,dict(active=True,per_second=rate,cost=h,normalized_lateral_error=z,
        command_age_ticks=updated_command_age_ticks,training_only=True)
