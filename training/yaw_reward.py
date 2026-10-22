"""Independent instantaneous straight-command yaw cost, training only."""
import math

def straight_yaw_precision(command,actual,updated_command_age_ticks):
    active=(math.sqrt(sum(float(v)**2 for v in command))>.04 and
            abs(float(command[2]))<=.04 and updated_command_age_ticks>=100)
    coefficient=4. if float(command[0])<-.04 else 2.
    if not active:
        return 0.,dict(active=False,per_second=0.,cost=None,coefficient=coefficient,command_age_ticks=updated_command_age_ticks,training_only=True)
    z=float(actual[2])/.2
    h=.5*z*z if abs(z)<1. else abs(z)-.5
    rate=-coefficient*h/(1.+h)
    return rate,dict(active=True,per_second=rate,cost=h,coefficient=coefficient,normalized_yaw_error=z,
        command_age_ticks=updated_command_age_ticks,training_only=True)
