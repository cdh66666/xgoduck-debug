"""Offline teacher for the official-reference geometry, not an accepted policy.

The waypoint/feedback parameters are simulated search results from 2026-10-26;
only the final seated geometry is upstream SitStand. No hardware interface.
Capture actual angles at each sit entry; the rise destination is separate.
"""
import numpy as np
from sit_training import HOME,smooth
from sit_reference import SEAT_DEG,SIT_DURATION,RISE_DURATION

SIT_PARAMETERS=(-3.7216077669295404,52.80612658223719,81.26284481209254,
    8.708633075793971,75.33568355500388,11.599848546981343,
    -23.043733507437754,-5.623001570700129,SIT_DURATION,.6,
    .28565291920768465,.21246068808771404,-3.4442843136729784,
    -.48897830766365064,.5667899705241447)
RISE_PARAMETERS=(9.657301719098456,75.88422462969663,80.90682618249086,
    7.945855215424481,55.66737115714501,55.81195527557001,
    -60.53564662373653,-31.489116229552096,RISE_DURATION,.3577455485236085,
    .2580059635295955,.13630413649133075,-5.371366429627812)


class SitCycleTeacher:
    def __init__(self,standing_deg):
        self.standing_deg=np.asarray(standing_deg,dtype=float).copy()
        self.sit_start=None

    def __call__(self,observation,current_joint_deg):
        progress=float(observation[55]);sitting=observation[56]>0
        if sitting and progress==0:
            self.sit_start=np.asarray(current_joint_deg,dtype=float).copy()
        if sitting and self.sit_start is None:
            raise ValueError('Sit teacher must start at phase zero to capture actual entry angles')
        p=SIT_PARAMETERS if sitting else RISE_PARAMETERS
        h1,k1,a1,h2,k2,a2,n1,n2,duration,split,kp,kd,goal=p[:13]
        mids=[]
        for h,k,a,n in ((h1,k1,a1,n1),(h2,k2,a2,n2)):
            q=SEAT_DEG.copy();q[[2,3,4,11,12,13]]=[h,k,a,-h,-k,-a];q[5]=n;mids.append(q)
        t=progress*duration
        times=[0,duration*split,duration*(.8 if sitting else .78),duration]
        poses=[self.sit_start,*mids,SEAT_DEG] if sitting else [SEAT_DEG,*mids,self.standing_deg]
        index=max(0,min(2,int(np.searchsorted(times,t,side='right')-1)))
        u=smooth((t-times[index])/(times[index+1]-times[index]))
        q=poses[index]+(poses[index+1]-poses[index])*u
        pitch=np.rad2deg(np.arctan2(observation[3],-observation[5]))
        fade=1-smooth((progress-.85)/.15) if sitting else smooth(t/.3)*(1-smooth((progress-.8)/.2))
        correction=np.clip(kp*(pitch-goal)+kd*np.rad2deg(observation[1]),-20,20)*fade
        q[4]=np.clip(q[4]+correction,-85,85);q[13]=np.clip(q[13]-correction,-85,85)
        action=np.deg2rad(q-HOME).astype(np.float32)
        if sitting:
            hip=np.deg2rad(np.clip(p[13]*pitch,-8,8))*smooth((progress-p[14])/(1-p[14]))
            action[2]+=hip;action[11]-=hip
        else:
            ankle=np.deg2rad(np.clip(.8*pitch+.15*np.rad2deg(observation[1]),-20,20))*smooth((progress-.65)/.35)
            action[4]+=ankle;action[13]-=ankle
        return action
