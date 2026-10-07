"""3D MuJoCo contact cross-check with actual source upper-body inertials and meshes."""
import json, pathlib, numpy as np, mujoco
from balance import gain,P
ROOT=pathlib.Path(__file__).resolve().parents[1]
def run(initial_deg=5,friction=None,torque=None,parked=False,poweroff=None,articulated=False,adaptive_com=False,push_N=0,driving=False):
 friction=P['ground_friction'] if friction is None else friction;torque=P['torque_limit_Nm_each'] if torque is None else torque
 path=ROOT/'simulation'/('parked.xml' if parked else 'articulated.xml' if articulated else 'balanced.xml')
 m=mujoco.MjModel.from_xml_path(str(path));d=mujoco.MjData(m);K,*_=gain();r=P['wheel_radius_mm']/1000;th=np.deg2rad(initial_deg)
 control_steps=max(1,round(P['control_period_s']/m.opt.timestep));delay_steps=round(P['torque_delay_s']/m.opt.timestep)
 # Root origin is axle; start just above ground to avoid artificial penetration.
 d.qpos[2]=r+.001;d.qpos[3:7]=[np.cos(th/2),0,np.sin(th/2),0]
 for g in ['left_tire','right_tire']:m.geom_friction[m.geom(g).id,0]=friction
 m.geom_friction[m.geom('floor').id,0]=friction
 for i in range(2):m.actuator_ctrlrange[i]=[-torque,torque]
 hold=d.qpos.copy(); logs=[];u=0;queue=[];max_roll=0;max_torque=0;floor_other=False;peak_arm=0
 ids=[i for i in range(1,m.nbody) if not m.body(i).name.endswith('_wheel')];mass=sum(m.body_mass[ids]);previous_com_pitch=th;com_pitch_rate=0
 progress=0;ref_progress=0;ref_yaw=0;peak_speed=0;peak_com_pitch=0;drive_logs=[];yaw_integral=0
 for step in range(8000):
  mujoco.mj_forward(m,d)
  quat=d.qpos[3:7];w,x,y,z=quat;pitch=np.arctan2(2*(w*y+x*z),1-2*(y*y+x*x));roll=np.arctan2(2*(w*x+y*z),1-2*(x*x+y*y));max_roll=max(max_roll,abs(roll))
  yaw=np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z));forward=np.array([np.cos(yaw),np.sin(yaw)])
  speed=float(d.qvel[:2]@forward)
  vref=.3*max(0,min((d.time-1),1,(5-d.time))) if driving else 0
  yaw_rate_ref=.3*max(0,min((d.time-2)/.5,1,(4.5-d.time)/.5)) if driving else 0
  if driving:progress+=speed*.001;ref_progress+=vref*.001;ref_yaw+=yaw_rate_ref*.001
  peak_speed=max(peak_speed,abs(speed))
  com=sum(m.body_mass[i]*d.xipos[i] for i in ids)/mass;vec=com-d.xpos[m.body('new_drive').id];com_pitch=float(np.arctan2(vec[:2]@forward,vec[2]));peak_com_pitch=max(peak_com_pitch,abs(com_pitch))
  if step%control_steps==0:
   com_pitch_rate=(com_pitch-previous_com_pitch)/(control_steps*m.opt.timestep) if step else d.qvel[4];previous_com_pitch=com_pitch
  s=np.array([progress-ref_progress if driving else d.qpos[0],speed-vref if driving else d.qvel[0],com_pitch if adaptive_com else pitch,com_pitch_rate if adaptive_com else d.qvel[4]])
  if step%control_steps==0:queue.append((step+delay_steps,float(np.clip(-K@s,-2*torque,2*torque))))
  while queue and queue[0][0]<=step:u=queue.pop(0)[1]
  if parked or (poweroff is not None and d.time>=poweroff):u=0
  common=u/2;delta=0
  if driving:
   # Desired world yaw torque -> wheel differential. Positive yaw is CCW about +Z.
   yaw_integral=float(np.clip(yaw_integral+(ref_yaw-yaw)*m.opt.timestep,-2,2))
   yaw_torque=12*(ref_yaw-yaw)+4*(yaw_rate_ref-d.sensor('body_gyro').data[2])+6*yaw_integral;delta=float(np.clip(yaw_torque*r/(P['wheel_track_mm']/1000),-torque+abs(common),torque-abs(common)))
  d.ctrl[0]=common-delta;d.ctrl[1]=common+delta;max_torque=max(max_torque,float(np.max(np.abs(d.ctrl[:2]))))
  d.xfrc_applied[m.body('waist_yaw_link').id,0]=push_N if 1<=d.time<1.15 else 0
  if articulated:
   for ai in range(2,m.nu):
    ji=m.actuator_trnid[ai,0];qi=m.jnt_qposadr[ji];vi=m.jnt_dofadr[ji];target=hold[qi]
    # Slow symmetric reach, source joint signs retained; no imitation biped policy.
    name=m.joint(ji).name
    if 'shoulder_pitch' in name:
     amp=.35*(1-np.cos(np.pi*np.clip((d.time-2)/2,0,1)))/2;target+=amp if 'left' in name else -amp
    d.ctrl[ai]=np.clip(120*(target-d.qpos[qi])-12*d.qvel[vi],*m.actuator_ctrlrange[ai]);peak_arm=max(peak_arm,abs(d.ctrl[ai]))
  if step%5==0:logs.append([d.time,*s,float(np.rad2deg(roll)),u/2])
  if driving and step%5==0:drive_logs.append([d.time,d.qpos[0],d.qpos[1],np.rad2deg(com_pitch),np.rad2deg(yaw),speed,*d.ctrl[:2],progress,ref_progress])
  for ci in range(d.ncon):
   c=d.contact[ci];names=[m.geom(int(g)).name for g in [c.geom1,c.geom2]]
   if 'floor' in names and not any(n in ['left_tire','right_tire'] or (n and n.startswith('parking_')) for n in names):floor_other=True
  if abs(pitch)>np.deg2rad(45) or abs(roll)>np.deg2rad(30):break
  mujoco.mj_step(m,d)
 recovered=bool(abs(com_pitch if adaptive_com else pitch)<np.deg2rad(1) and abs(d.qvel[4])<.02 and d.time>7.9)
 if driving:recovered=recovered and abs(progress-ref_progress)<.03 and abs(ref_yaw-yaw)<np.deg2rad(1)
 result={'initial_deg':initial_deg,'friction':friction,'torque_Nm_each':torque,'parked':parked,'poweroff_s':poweroff,'articulated':articulated,'adaptive_com':adaptive_com,'driving':driving,'torso_push_N_for_150ms':push_N,'duration_s':d.time,'final_pitch_deg':float(np.rad2deg(pitch)),'final_COM_pitch_deg':float(np.rad2deg(com_pitch)),'max_COM_pitch_deg':float(np.rad2deg(peak_com_pitch)),'max_roll_deg':float(np.rad2deg(max_roll)),'peak_speed_m_s':peak_speed,'final_yaw_deg':float(np.rad2deg(yaw)),'reference_yaw_deg':float(np.rad2deg(ref_yaw)),'final_progress_m':progress,'reference_progress_m':ref_progress,'peak_torque_Nm_each':max_torque,'peak_upper_joint_torque_Nm':peak_arm,'final_x_m':float(d.qpos[0]),'final_y_m':float(d.qpos[1]),'recovered':bool(recovered),'nonwheel_floor_contact':floor_other,'model_limits':'cylinder tyres, assumed wheel inertia, no thermal/electrical motor dynamics; source capsules are approximate collision geometry'}
 if driving:np.savetxt(ROOT/'checks/driving_trace.csv',drive_logs,delimiter=',',header='time_s,x_m,y_m,COM_pitch_deg,yaw_deg,forward_speed_m_s,left_torque_Nm,right_torque_Nm,path_distance_m,reference_distance_m',comments='')
 return result,np.array(logs)
if __name__=='__main__':
 results=[]
 for angle,mu,t in [(2,.6,6),(5,.6,6),(10,.6,6),(15,.6,6),(5,.2,6),(10,.1,6),(5,.6,1)]:
  a,tr=run(angle,mu,t);results.append(a);print(json.dumps(a),flush=True)
  if angle==5 and mu==.6 and t==6:np.savetxt(ROOT/'checks/contact_trace.csv',tr,delimiter=',',header='time_s,x_m,v_m_s,pitch_rad,pitch_rate_rad_s,roll_deg,torque_Nm_each',comments='')
 for args in [dict(initial_deg=0,parked=True),dict(initial_deg=2,poweroff=1),dict(initial_deg=5,articulated=True),dict(initial_deg=5,articulated=True,adaptive_com=True),dict(initial_deg=2,push_N=50),dict(initial_deg=2,push_N=100),dict(initial_deg=2,articulated=True,adaptive_com=True,driving=True)]:
  a,_=run(**args);results.append(a);print(json.dumps(a),flush=True)
 (ROOT/'checks/contact_results.json').write_text(json.dumps(results,indent=2))
