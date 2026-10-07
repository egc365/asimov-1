"""Nonlinear wheeled inverted pendulum, source-derived combined mass and inertia.
Units: metre, second, kg, radian, Nm. Torque is total of both motors.
Wheel inertia is expressed in world wheel rotation; motor reaction acts on body.
This is a planar screening model; contact cross-check lives in contact_check.py.
"""
import json, pathlib, numpy as np
from scipy.linalg import solve_continuous_are
ROOT=pathlib.Path(__file__).resolve().parents[1]
P=json.loads((ROOT/'parameters.json').read_text())
PROPS=json.loads((ROOT/'checks/mass_properties.json').read_text())
def derivative(s,u,props=PROPS,slope=0,force=0):
 x,v,th,w=s; m=props['body_mass_kg'];h=props['com_above_axle_m'][2];r=props['wheel_radius_m'];J=props['body_pitch_inertia_COM_kgm2'];mw=props['wheel_mass_kg_each']
 # Two solid wheels: added translational inertia 2 I/r^2 = mw.
 A=m+3*mw; B=m*h*np.cos(th); C=J+m*h*h
 rhs=np.array([u/r+m*h*np.sin(th)*w*w-(m+2*mw)*9.81*np.sin(slope)+force,m*9.81*h*np.sin(th)-u])
 acc=np.linalg.solve([[A,B],[B,C]],rhs)
 return np.array([v,acc[0],w,acc[1]])
def gain(props=PROPS):
 eps=1e-5; zero=np.zeros(4)
 A=np.column_stack([(derivative(np.eye(4)[i]*eps,0,props)-derivative(-np.eye(4)[i]*eps,0,props))/(2*eps) for i in range(4)])
 B=((derivative(zero,eps,props)-derivative(zero,-eps,props))/(2*eps)).reshape(4,1)
 Q=np.diag([10,8,400,30]);R=np.array([[.08]])
 S=solve_continuous_are(A,B,Q,R);K=np.linalg.solve(R,B.T@S)
 return K[0],A,B,np.linalg.eigvals(A-B@K)
def simulate(initial_deg=5,limit_each=None,delay=None,period=None,friction=None,body_scale=1,wheel_radius=None,com_shift=0,slope_deg=0,push_N=0,motor_off_at=None,duration=8,save=False):
 limit_each=P['torque_limit_Nm_each'] if limit_each is None else limit_each
 delay=P['torque_delay_s'] if delay is None else delay;period=P['control_period_s'] if period is None else period
 friction=P['ground_friction'] if friction is None else friction;wheel_radius=PROPS['wheel_radius_m'] if wheel_radius is None else wheel_radius
 props=dict(PROPS);props['body_mass_kg']*=body_scale;props['body_pitch_inertia_COM_kgm2']*=body_scale;props['com_above_axle_m']=list(PROPS['com_above_axle_m']);props['com_above_axle_m'][2]+=com_shift;props['wheel_radius_m']=wheel_radius
 K,*_=gain(PROPS) # Deliberately fixed nominal controller for robustness screening.
 dt=.001;s=np.array([0,0,np.deg2rad(initial_deg),0.]);hist=[];queue=[];u=0;next_control=0;peak=0;slips=0;fall=False;slope=np.deg2rad(slope_deg)
 for step in range(int(duration/dt)):
  t=step*dt
  if t>=next_control-1e-10:
   target=float(-K@s); clipped=float(np.clip(target,-2*limit_each,2*limit_each));queue.append((t+delay,clipped));next_control+=period
  while queue and queue[0][0]<=t+1e-10:u=queue.pop(0)[1]
  if motor_off_at is not None and t>=motor_off_at:u=0
  # Traction is an upper bound; planar model does not simulate tyre slip.
  required_force=abs(u/wheel_radius);traction=friction*(props['body_mass_kg']+2*props['wheel_mass_kg_each'])*9.81
  if required_force>traction:slips+=1;u=float(np.clip(u,-traction*wheel_radius,traction*wheel_radius))
  force=push_N if 1<=t<1.15 else 0
  f=lambda a:derivative(a,u,props,slope,force)
  k1=f(s);k2=f(s+dt*k1/2);k3=f(s+dt*k2/2);k4=f(s+dt*k3);s+=dt*(k1+2*k2+2*k3+k4)/6
  peak=max(peak,abs(u)/2)
  if save and step%5==0:hist.append([t,*s,u/2])
  if abs(s[2])>np.deg2rad(45):fall=True;break
 result={'initial_deg':initial_deg,'torque_limit_Nm_each':limit_each,'delay_s':delay,'control_period_s':period,'friction':friction,'body_mass_scale':body_scale,'radius_m':wheel_radius,'com_delta_m':com_shift,'slope_deg':slope_deg,'push_N_150ms':push_N,'motor_off_at_s':motor_off_at,'recovered':bool(not fall and abs(s[2])<np.deg2rad(1) and abs(s[3])<.02),'fall_45deg':fall,'duration_s':round(t,3),'final_pitch_deg':float(np.rad2deg(s[2])),'final_x_m':float(s[0]),'peak_torque_Nm_each':peak,'traction_clipped_steps':slips,'planar_limit':'normal force quasi-static; no tyre slip, roll, collision or motor thermal dynamics'}
 return result,np.array(hist)
if __name__=='__main__':
 K,A,B,eig=gain();(ROOT/'checks/controller.json').write_text(json.dumps({'state_order':['x_m','v_m_s','pitch_rad','pitch_rate_rad_s'],'K_total_torque':K.tolist(),'linear_closed_loop_eigenvalues':[[float(z.real),float(z.imag)] for z in eig],'control_period_s':P['control_period_s'],'release':'simulation only; no hardware torque calibration'},indent=2))
 (ROOT/'simulation/controller_gains.h').write_text('/* Generated from checks/mass_properties.json by balance.py; do not hand edit. */\n#ifndef CONTROLLER_GAINS_H\n#define CONTROLLER_GAINS_H\n'+''.join(f'#define BALANCE_K_{i} ({-v:.17g})\n' for i,v in enumerate(K))+f'#define BALANCE_TORQUE_EACH_MAX ({P["torque_limit_Nm_each"]:.17g})\n#endif\n')
 rows=[]
 for angle in [2,5,10,15,20,25,30]:
  for torque in [1,2,3,6]:rows.append(simulate(initial_deg=angle,limit_each=torque)[0])
 for delay in [0,.005,.01,.02,.04,.06,.08,.1]:rows.append(simulate(delay=delay)[0])
 for friction in [.1,.2,.3,.4,.6]:rows.append(simulate(friction=friction,initial_deg=10)[0])
 for scale in [.8,1,1.2,1.4]:rows.append(simulate(body_scale=scale)[0])
 for radius in [.06,.07,.08,.09,.1]:rows.append(simulate(wheel_radius=radius)[0])
 for shift in [-.1,.1,.2]:rows.append(simulate(com_shift=shift)[0])
 for slope in [0,3,6,10]:rows.append(simulate(slope_deg=slope)[0])
 for push in [20,50,100,200]:rows.append(simulate(push_N=push)[0])
 rows.append(simulate(motor_off_at=1,push_N=10)[0])
 rows.append(simulate(period=.1,delay=.05)[0])
 nominal,traj=simulate(save=True);np.savetxt(ROOT/'checks/nominal_trace.csv',traj,delimiter=',',header='time_s,x_m,v_m_s,pitch_rad,pitch_rate_rad_s,torque_Nm_each',comments='')
 (ROOT/'checks/planar_sweep.json').write_text(json.dumps(rows,indent=2));print(json.dumps({'cases':len(rows),'recovered':sum(r['recovered'] for r in rows),'nominal':nominal,'K':K.tolist()},indent=2))
