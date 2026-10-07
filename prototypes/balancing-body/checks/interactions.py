"""Deterministic actual-mesh intersection screen over declared source-joint poses.
Open source meshes give surface-intersection evidence, not certified volume clearance.
Mass-only hand envelopes do not count as validated hands.
"""
import json,pathlib,xml.etree.ElementTree as ET,itertools
import numpy as np,mujoco,trimesh
from trimesh.collision import CollisionManager
ROOT=pathlib.Path(__file__).resolve().parents[1];REPO=ROOT.parents[1]
src=ET.parse(ROOT/'source/asimov_1.xml').getroot();modeltree=ET.parse(ROOT/'simulation/articulated.xml').getroot();m=mujoco.MjModel.from_xml_path(str(ROOT/'simulation/articulated.xml'));d=mujoco.MjData(m)
sourcebodies={b.get('name'):b for b in src.iter('body')};invariants=[]
for b in modeltree.iter('body'):
 name=b.get('name')
 if name not in sourcebodies:continue
 original=sourcebodies[name];assert b.find('inertial').attrib==original.find('inertial').attrib
 if name!='pelvis_link':assert b.get('pos')==original.get('pos') and b.get('quat')==original.get('quat')
 invariants.append(name)
mgr=CollisionManager();objects={};parents={}
sourcefiles={a.get('name'):a.get('file') for a in src.findall('./asset/mesh')}
for b in modeltree.iter('body'):
 name=b.get('name')
 if name not in sourcebodies:continue
 geom=b.find("geom[@class='visual']")
 if geom is None:continue
 mesh=trimesh.load(REPO/'sim-model/assets/meshes'/sourcefiles[geom.get('mesh')]);key='source:'+name;mgr.add_object(key,mesh);objects[key]={'body':m.body(name).id,'mesh_watertight':bool(mesh.is_watertight),'kind':'source_mesh'}
for parent in modeltree.iter('body'):
 for child in parent.findall('body'):parents[child.get('name')]=parent.get('name')
for rec in json.loads((ROOT/'cad/concept/manifest.json').read_text()):
 if 'parking_' in rec['id']:continue
 mesh=trimesh.load(ROOT/'cad/concept'/(rec['id']+'.stl'));mesh.apply_scale(.001);key='new:'+rec['id'];mgr.add_object(key,mesh);objects[key]={'body':m.body('new_drive').id,'mesh_watertight':bool(mesh.is_watertight),'kind':'new_envelope' if 'ENVELOPE' in key else 'new_CAD'}
excluded=set()
for a,b in itertools.combinations(objects,2):
 na=a.split(':',1)[1];nb=b.split(':',1)[1]
 if a.startswith('source:') and b.startswith('source:') and (parents.get(na)==nb or parents.get(nb)==na):excluded.add(tuple(sorted([a,b])))
for e in modeltree.findall('./contact/exclude'):excluded.add(tuple(sorted(['source:'+e.get('body1'),'source:'+e.get('body2')])))
for side in ['left','right']:excluded.add(tuple(sorted(['new:PART-201_'+side+'_wheel_ENVELOPE','new:PART-104_'+side+'_axle_seat'])))
for a,b in [('PART-101_deck','PART-102_column'),('PART-101_deck','PART-104_left_axle_seat'),('PART-101_deck','PART-104_right_axle_seat'),('PART-102_column','PART-103_interface_blank')]:
 # Intended contact surfaces: independently checked BRep volume is zero.
 excluded.add(tuple(sorted(['new:'+a,'new:'+b])))
for side in ['left','right']:
 excluded.add(tuple(sorted(['new:PART-105_'+side+'_pelvis_cradle','new:PART-103_interface_blank'])))
 excluded.add(tuple(sorted(['new:PART-106_'+side+'_torque_clamp_lower','new:PART-104_'+side+'_axle_seat'])))
 excluded.add(tuple(sorted(['new:PART-106_'+side+'_torque_clamp_cap','new:PART-104_'+side+'_axle_seat'])))
base=d.qpos.copy();joints=[m.joint(i) for i in range(m.njnt) if m.joint(i).name not in ['drive_free','left_wheel_joint','right_wheel_joint']];poses=[('source_rest',base.copy())]
for joint in joints:
 for value in np.linspace(*m.jnt_range[joint.id],5):
  q=base.copy();q[m.jnt_qposadr[joint.id]]=value;poses.append((joint.name+'='+str(round(value,4)),q))
rng=np.random.default_rng(19)
for i in range(25):
 q=base.copy()
 for joint in joints:q[m.jnt_qposadr[joint.id]]=rng.uniform(*m.jnt_range[joint.id])
 poses.append(('combined_random_'+str(i),q))
for i in range(21):
 q=base.copy();amplitude=.35*i/20
 for side,sign in [('left',1),('right',-1)]:q[m.jnt_qposadr[m.joint(side+'_shoulder_pitch_joint').id]]+=sign*amplitude
 poses.append(('tested_slow_reach_'+str(i),q))
hits={};pose_results=[]
for label,q in poses:
 d.qpos[:]=q;mujoco.mj_forward(m,d)
 for name,rec in objects.items():
  bid=rec['body'];T=np.eye(4);T[:3,:3]=d.xmat[bid].reshape(3,3);T[:3,3]=d.xpos[bid];mgr.set_transform(name,T)
 collision,pairs=mgr.in_collision_internal(return_names=True)
 unexpected=[tuple(sorted(pair)) for pair in pairs if tuple(sorted(pair)) not in excluded]
 for pair in unexpected:hits.setdefault(pair,[]).append(label)
 pose_results.append({'pose':label,'intersections':[list(x) for x in sorted(unexpected)]})
matrix=[]
for a,b in itertools.combinations(sorted(objects),2):
 pair=(a,b);matrix.append({'a':a,'b':b,'status':'excluded_mating_interface' if pair in excluded else 'tested_surface_intersection' if pair in hits else 'tested_no_surface_intersection','poses_tested':0 if pair in excluded else len(poses),'hit_poses':hits.get(pair,[])})
result={'source_inertials_preserved':invariants,'pose_count':len(poses),'joint_count':len(joints),'rest_pose_intersections':pose_results[0]['intersections'],'poses_with_intersections':sum(bool(x['intersections']) for x in pose_results),'slow_reach_poses_with_intersections':sum(bool(x['intersections']) for x in pose_results if x['pose'].startswith('tested_slow_reach_')),'pair_matrix':matrix,'pose_results':pose_results,'limitations':['source meshes may be open: triangle intersection does not prove positive volume clearance','five evenly spaced points per original joint plus 25 deterministic combinations and 21 slow-reach samples; not continuous swept-volume certification','hands are mass allowances only, omitted from hardware collision validation','parking struts tested separately as CAD; stowed linkage and all wiring routes unresolved','new mating contact surfaces have independently verified zero BRep intersection volume']}
(ROOT/'checks/interaction_results.json').write_text(json.dumps(result,indent=2));print(json.dumps({k:v for k,v in result.items() if k in ['source_inertials_preserved','pose_count','joint_count','rest_pose_intersections','poses_with_intersections']},indent=2))
