"""Keep source upper-body inertials and link geometry; replace leg subtrees with custom drive."""
import copy, json, pathlib, xml.etree.ElementTree as ET
import os
import numpy as np, mujoco
ROOT=pathlib.Path(__file__).resolve().parents[1]
REPO=ROOT.parents[1]
P=json.loads((ROOT/'parameters.json').read_text())
def build(articulated=False,parked=False,extra_payload=0):
 r=ET.parse(ROOT/'source/asimov_1.xml').getroot()
 r.set('model','Asimov_upper_on_new_balancing_base')
 r.find('compiler').set('meshdir',str(REPO/'sim-model/assets/meshes'))
 r.find('compiler').set('fusestatic','false')
 r.find('option').set('timestep','.001')
 r.find('option').set('iterations','80')
 r.find('option').set('ls_iterations','20')
 # New studio rendering assets; physical source collision geometry remains intact.
 asset=r.find('asset')
 for e in list(asset):
  if e.tag in ('texture','material'): asset.remove(e)
 for name,color in [('visual_grey','.79 .81 .77 1'),('collision_grey','.5 .5 .5 1'),('groundplane','.91 .92 .90 1'),('material','.8 .8 .8 1'),('collision_material','1 .3 .1 1')]: ET.SubElement(asset,'material',name=name,rgba=color)
 ET.SubElement(asset,'texture',name='studio_sky',type='skybox',builtin='gradient',rgb1='.68 .73 .75',rgb2='.90 .92 .92',width='512',height='3072')
 world=r.find('worldbody'); upper=world.find('body'); world.remove(upper)
 for e in list(world):
  if e.tag=='light':world.remove(e)
 floor=world.find('geom');floor.set('friction',f"{P['ground_friction']} .005 .0001");floor.set('rgba','.91 .92 .90 1')
 for pos,di,df in [('1 -2 3','-1 2 -3','.9 .9 .9'),('-2 1 2','2 -1 -2','.5 .5 .5'),('0 2 3','0 -2 -3','.6 .6 .6')]:ET.SubElement(world,'light',pos=pos,dir=di,diffuse=df,castshadow='true')
 for child in list(upper):
  if child.tag=='body' and child.get('name').startswith(('left_hip','right_hip')): upper.remove(child)
  elif child.tag in ('freejoint','camera'): upper.remove(child)
 placement=json.loads((ROOT/'cad/concept/placement.json').read_text());upper.set('pos',' '.join(str(v/1000) for v in placement['pelvis_origin_mm']))
 for b in upper.iter('body'):
  for j in list(b.findall('joint')):
   if not articulated:b.remove(j)
   else:
    j.set('damping','1');j.set('stiffness','0')
 for side in ['left','right']:
  wrist=next(b for b in upper.iter('body') if b.get('name')==side+'_wrist_yaw_link')
  hand=ET.SubElement(wrist,'body',name=side+'_hand_allowance',pos='.04 0 -.04')
  ET.SubElement(hand,'inertial',mass=str(P['hand_allowance_kg_each']+extra_payload/2),pos='0 0 0',diaginertia='.001 .001 .001')
  ET.SubElement(hand,'geom',name=side+'_hand_allowance_geom',type='box',size='.045 .035 .06',rgba='.12 .35 .31 1',contype='0',conaffinity='0',group='2')
 base=ET.SubElement(world,'body',name='new_drive',pos=f"0 0 {P['wheel_radius_mm']/1000+.001}")
 ET.SubElement(base,'freejoint',name='drive_free')
 records=json.loads((ROOT/'cad/concept/manifest.json').read_text())
 for rec in records:
  name=rec['id']
  if 'wheel_' in name or 'parking_' in name: continue
  mesh=ET.SubElement(asset,'mesh',name=name,file=str(ROOT/'cad/concept'/(name+'.stl')),scale='.001 .001 .001')
  b=ET.SubElement(base,'body',name=name)
  mass=rec['mass_kg']; com=np.array(rec['com_mm'])/1000; size=np.array(rec['bbox_mm'])/1000
  # Solid box approximation for new component inertias; source link tensors are exact source values.
  inertia=mass/12*np.array([size[1]**2+size[2]**2,size[0]**2+size[2]**2,size[0]**2+size[1]**2])
  ET.SubElement(b,'inertial',pos=' '.join(map(str,com)),mass=str(mass),diaginertia=' '.join(map(str,inertia)))
  ET.SubElement(b,'geom',type='mesh',mesh=name,rgba='.14 .24 .28 1' if 'ENVELOPE' not in name else '.12 .35 .31 1',contype='0',conaffinity='0',group='2')
  bounds=np.array(rec['bbox_bounds_mm'])/1000;center=(bounds[:3]+bounds[3:])/2
  ET.SubElement(b,'geom',name=name+'_collision',type='box',size=' '.join(map(str,size/2)),pos=' '.join(map(str,center)),group='3',rgba='.2 .6 .2 .3')
 parking_mass=sum(rec['mass_kg'] for rec in records if 'parking_' in rec['id'])
 for name,mass,pos in [('electronics',P['electronics_mass_kg'],[0,0,.14]),('fasteners_cables',P['fasteners_cables_mass_kg'],[0,0,.15]),('stowed_parking',parking_mass,[0,0,.095])]:
  b=ET.SubElement(base,'body',name=name,pos=' '.join(map(str,pos)))
  ET.SubElement(b,'inertial',mass=str(mass),pos='0 0 0',diaginertia='.002 .002 .002')
 base.append(upper)
 actuator=ET.SubElement(r,'actuator')
 radius=P['wheel_radius_mm']/1000; width=P['wheel_width_mm']/1000
 for side,sign in [('left',1),('right',-1)]:
  wh=ET.SubElement(base,'body',name=side+'_wheel',pos=f"0 {sign*P['wheel_track_mm']/2000} 0")
  ET.SubElement(wh,'joint',name=side+'_wheel_joint',axis='0 1 0',type='hinge',limited='false',damping='.005',armature='0')
  mw=P['wheel_mass_kg_each']; iy=.5*mw*radius**2; ix=mw*(3*radius**2+width**2)/12
  ET.SubElement(wh,'inertial',mass=str(mw),pos='0 0 0',diaginertia=f'{ix} {iy} {ix}')
  ET.SubElement(wh,'geom',name=side+'_tire',type='cylinder',size=f'{radius} {width/2}',quat='.70710678 .70710678 0 0',rgba='.045 .055 .065 1',friction=f"{P['ground_friction']} .005 .0001",condim='3',priority='2')
  ET.SubElement(wh,'geom',type='cylinder',size=f'{radius*.48} {width/2+.001}',quat='.70710678 .70710678 0 0',rgba='.55 .59 .59 1',contype='0',conaffinity='0',group='2')
  ET.SubElement(base,'geom',name=side+'_shaft_envelope',type='cylinder',size=f"{P['shaft_diameter_mm']/2000} .025",pos=f'0 {sign*.172} 0',quat='.70710678 .70710678 0 0',rgba='.55 .59 .59 1',contype='0',conaffinity='0',group='2',density='0')
  ET.SubElement(actuator,'motor',name=side+'_drive_torque',joint=side+'_wheel_joint',gear='1',ctrllimited='true',ctrlrange=f"{-P['torque_limit_Nm_each']} {P['torque_limit_Nm_each']}")
 # Prune references to removed legs; retain source upper-body collision excludes.
 contact=r.find('contact')
 names={b.get('name') for b in r.iter('body')}
 for ex in list(contact):
  if ex.get('body1') not in names or ex.get('body2') not in names:contact.remove(ex)
 if articulated:
  for joint in upper.iter('joint'):
   name=joint.get('name');limit=30 if 'shoulder_pitch' in name else 25 if 'shoulder_roll' in name else 20 if 'shoulder_yaw' in name else 40 if 'waist' in name else 12
   ET.SubElement(actuator,'motor',name=name+'_hold',joint=name,gear='1',ctrllimited='true',ctrlrange=f'{-limit} {limit}')
 # CAD and model share the same source-derived cradle placement; no hidden repositioning.
 used_meshes={geom.get('mesh') for geom in r.iter('geom') if geom.get('mesh')}
 for mesh in list(asset.findall('mesh')):
  if mesh.get('name') not in used_meshes:asset.remove(mesh)
 model=mujoco.MjModel.from_xml_string(ET.tostring(r,encoding='unicode')); data=mujoco.MjData(model);mujoco.mj_forward(model,data)
 ids=[i for i in range(1,model.nbody) if not model.body(i).name.endswith('_wheel')]
 mass=sum(model.body_mass[ids]); cx=sum(model.body_mass[i]*data.xipos[i,0] for i in ids)/mass
 if extra_payload==0:assert abs(cx)<1e-8,'CAD/model COM or source neutral-frame mismatch'
 if parked:
  for rec in records:
   if 'parking_' in rec['id']:
    name=rec['id'];ET.SubElement(asset,'mesh',name=name,file=str(ROOT/'cad/concept'/(name+'.stl')),scale='.001 .001 .001');ET.SubElement(base,'geom',type='mesh',mesh=name,contype='0',conaffinity='0',group='2',rgba='.55 .60 .61 1')
  for x in [-.18,.18]:
   for y in [-.13,.13]:ET.SubElement(base,'geom',name=f'parking_{x}_{y}',type='capsule',fromto=f'{x} {y} .09 {x} {y} {-radius+.008}',size='.008',rgba='.6 .63 .65 1',friction=f"{P['ground_friction']} .005 .0001",group='3')
 ET.SubElement(base,'site',name='balance_imu',pos='0 0 .12',size='.005')
 sensor=ET.SubElement(r,'sensor');ET.SubElement(sensor,'gyro',name='body_gyro',site='balance_imu');ET.SubElement(sensor,'accelerometer',name='body_accel',site='balance_imu');ET.SubElement(sensor,'framequat',name='body_orientation',objtype='site',objname='balance_imu')
 # Portable assets: source files remain in their repository location; new CAD in this project.
 meshroot=REPO/'sim-model/assets/meshes'
 for mesh in asset.findall('mesh'):
  if pathlib.Path(mesh.get('file')).is_absolute():mesh.set('file',os.path.relpath(mesh.get('file'),meshroot))
 r.find('compiler').set('meshdir','../../../sim-model/assets/meshes')
 path=ROOT/'simulation'/('articulated.xml' if articulated else 'parked.xml' if parked else 'balanced.xml');ET.indent(r);ET.ElementTree(r).write(path,encoding='unicode')
 return path
def properties(path):
 m=mujoco.MjModel.from_xml_path(str(path));d=mujoco.MjData(m);mujoco.mj_forward(m,d)
 ids=[i for i in range(1,m.nbody) if not m.body(i).name.endswith('_wheel')];mass=float(sum(m.body_mass[ids])); axle=d.xpos[m.body('new_drive').id]
 c=sum(m.body_mass[i]*d.xipos[i] for i in ids)/mass;I=np.zeros((3,3))
 for i in ids:
  rot=d.ximat[i].reshape(3,3);v=d.xipos[i]-c;I+=rot@np.diag(m.body_inertia[i])@rot.T+m.body_mass[i]*(np.eye(3)*np.dot(v,v)-np.outer(v,v))
 upper=m.body('pelvis_link').id
 return {'body_mass_kg':mass,'total_mass_kg':float(sum(m.body_mass)),'com_above_axle_m':(c-axle).tolist(),'body_pitch_inertia_COM_kgm2':float(I[1,1]),'wheel_radius_m':P['wheel_radius_mm']/1000,'wheel_track_m':P['wheel_track_mm']/1000,'wheel_mass_kg_each':P['wheel_mass_kg_each'],'source_upper_body_kg':19.332296427812093,'upper_with_hand_allowances_kg':float(m.body_subtreemass[upper]),'upper_COM_above_axle_m':(d.subtree_com[upper]-axle).tolist(),'new_inertias':'source tensors preserved; new parts use bbox solid-box approximation, wheel solid-cylinder assumption','new_collisions':'bounding boxes for new base components, original capsules for upper body; exact static base BRep intersections checked separately'}
if __name__=='__main__':
 p=build();build(articulated=True);build(parked=True); props=properties(p);(ROOT/'checks/mass_properties.json').write_text(json.dumps(props,indent=2));print(json.dumps(props,indent=2))
