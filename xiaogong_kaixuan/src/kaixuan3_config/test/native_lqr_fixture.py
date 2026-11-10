"""Generate independent Python numerical reference for the C++ servo; offline only."""
from pathlib import Path
import sys
import numpy as np
import yaml
PACKAGE=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(PACKAGE/'scripts'))
from cst_cartesian_math import RobotModel,JOINTS
from cst_cartesian_lqr import LQRTorqueFeedback,design_schedule
c=yaml.safe_load((PACKAGE/'config/cst_cartesian_lqr.yaml').read_text())
c.update(runtime_backend='ros2_control',control_hz=500.,velocity_filter_s=0.)
c['lqr'].update(runtime_hz=1000.,input_dynamics=dict(joint='left_joint4',delay_steps=3,time_constant_s=.0034,state_effort_weight=.01))
if len(sys.argv)>2 and sys.argv[2]=='multi':
    c['lqr']['input_dynamics']={'channels':[dict(joint='left_joint3',delay_steps=2,time_constant_s=.0044,state_effort_weight=.01),dict(joint='left_joint4',delay_steps=3,time_constant_s=.0021,state_effort_weight=.01)]}
m=RobotModel((PACKAGE.parent/'xiaogong_kaixuan/urdf/xiaogong-kaixuan.urdf').read_text(),c,np.zeros(7))
r=yaml.safe_load((PACKAGE.parent/'kaixuan_can_hardware/config/cartesian_torque_limits.yaml').read_text())['joints']
limits={k:np.array([r[j][k] for j in JOINTS]) for k in ['effort_limit_nm','effort_slew_nm_s']}
limits['effort_release_slew_nm_s']=limits['effort_slew_nm_s'].copy();limits['effort_release_slew_nm_s'][3]*=4
f=LQRTorqueFeedback(m,c,limits);qr=np.deg2rad(c['target_joints_deg']);zero=np.zeros(7)
f.schedules={str(i):design_schedule(m,lambda t,i=i:(qr+.01*i,zero,zero),0.,c) for i in range(3)}
rng=np.random.default_rng(4321);steps=4000
settings=c['lqr']['input_dynamics'];channels=settings.get('channels',[settings])
metadata=np.array([[JOINTS.index(s['joint']),s['delay_steps'],s['time_constant_s']] for s in channels]).ravel()
data=[np.r_[steps,len(channels),metadata,limits['effort_limit_nm'],limits['effort_slew_nm_s'],limits['effort_release_slew_nm_s'],c['integral_limit_nm']]]
for k in range(steps):
 phase=str((k//700)%3);scale=min(k/300.,1.) if k<3500 else max(0.,1.-(k-3500)/500.)
 dt=.001*(1.+.1*np.sin(k*.11));vr=rng.normal(0,.02,7);ar=rng.normal(0,.1,7)
 q=qr+.1*np.sin(k*.009+np.arange(7)) if k<2500 else qr+np.ones(7)
 v=rng.normal(0,.15,7)
 f.select(phase,k*.001);command,raw,integ=f.step(q,v,(qr,vr,ar),dt,scale)
 data.append(np.r_[2,int(phase),scale,dt,qr,vr,f.feedforward,q,v,f.gain.ravel(),f.inverse_integral_gain.ravel(),command,raw,integ,f.z,f.state_feedback,f.memory_feedback,f.input_model.state])
np.concatenate(data).astype('<f8').tofile(sys.argv[1]);print(f'Wrote {steps} offline coupled LQI reference steps')
