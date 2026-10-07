"""Editable, millimetre-native CadQuery design; never scale source actuator CAD."""
import json, pathlib, sys
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
import cadquery as cq
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
ROOT=pathlib.Path(__file__).resolve().parents[1]
P=json.loads((ROOT/'parameters.json').read_text())
OUT=ROOT/'cad/concept'; OUT.mkdir(exist_ok=True)
def validate(p):
    if p['wheel_radius_mm'] < 60: raise ValueError('wheel envelope below studied range')
    if p['column_wall_mm'] <= 0 or 2*p['column_wall_mm'] >= min(p['column_width_mm'],p['column_depth_mm']): raise ValueError('invalid tube wall')
    if p['shaft_bore_mm'] <= p['shaft_diameter_mm']: raise ValueError('shaft clearance must be positive')
    if p['wheel_track_mm']-p['wheel_width_mm'] < p['deck_width_mm']+20: raise ValueError('wheel/deck clearance insufficient')
    if p['deck_thickness_mm'] < 8: raise ValueError('deck below current welded-frame screen')
validate(P)
parts=[]
def add(name,shape,material='aluminum',fixed_mass=None):
    cq.exporters.export(shape,str(OUT/(name+'.step')))
    cq.exporters.export(shape,str(OUT/(name+'.stl')),tolerance=.3,angularTolerance=.15)
    mass=shape.val().Volume()*1e-9*P['aluminum_density_kg_m3'] if fixed_mass is None else fixed_mass
    parts.append((name,shape,material,mass))
z=P['deck_z_mm']; t=P['deck_thickness_mm']
deck=cq.Workplane('XY').box(P['deck_length_mm'],P['deck_width_mm'],t).edges('|Z').fillet(8).faces('>Z').workplane().pushPoints([(x,y) for x in [-85,85] for y in [-120,120]]).hole(6.6).translate((0,0,z))
add('PART-101_deck',deck)
col=cq.Workplane('XY').rect(P['column_depth_mm'],P['column_width_mm']).rect(P['column_depth_mm']-2*P['column_wall_mm'],P['column_width_mm']-2*P['column_wall_mm']).extrude(P['column_height_mm']).translate((0,0,z+t/2))
add('PART-102_column',col)
adapter=cq.Workplane('XY').box(160,160,6).faces('>Z').workplane().pushPoints([(x,y) for x in [-55,55] for y in [-55,55]]).hole(6.6).translate((0,0,z+t/2+P['column_height_mm']+3))
add('PART-103_interface_blank',adapter)
# Retained source hip mounting frames are explicit interfaces, not the plastic underside.
# Project the actual outer eight 4.3mm holes from the source left-hip mounting plate.
sourceplate=cq.importers.importStep(str(ROOT/'source/ASV1_200_03A.STEP')).val()
assert sourceplate.isValid()
projected=set()
for edge in sourceplate.Edges():
 if edge.geomType()=='CIRCLE' and abs(edge.radius()-2.15)<1e-6:
  center=edge.arcCenter();x=center.x;zz=center.z-109.8
  if abs(np.hypot(x,zz)-50.1)<.001:projected.add((round(x,6),round(zz,6)))
assert len(projected)==8
base_top=z+t/2+P['column_height_mm']+6
hip_z=P['pelvis_z_mm']-44.045;lower=hip_z-base_top
cheek=cq.Workplane('XZ').rect(120,lower+55).extrude(4,both=True).translate((0,0,(55-lower)/2))
for x,zz in projected:cheek=cheek.cut(cq.Workplane('XZ').center(x,zz).circle(2.15).extrude(20,both=True))
# Source rest posture is qpos=joint ref, so body quaternions and positions give neutral FK.
source=ET.parse(ROOT/'source/asimov_1.xml').getroot().find('./worldbody/body');moments=[]
def neutral(body,pos,rotation,isroot=False):
 if not isroot:pos=pos+rotation@np.fromstring(body.get('pos','0 0 0'),sep=' ')
 q=np.fromstring(body.get('quat','1 0 0 0'),sep=' ');rot=rotation@Rotation.from_quat([q[1],q[2],q[3],q[0]]).as_matrix()
 inertia=body.find('inertial');m=float(inertia.get('mass'));com=pos+rot@np.fromstring(inertia.get('pos'),sep=' ');moments.append((m,com))
 if body.get('name').endswith('_wrist_yaw_link'):moments.append((P['hand_allowance_kg_each'],pos+rot@np.array([.04,0,-.04])))
 for child in body.findall('body'):
  if not child.get('name').startswith(('left_hip','right_hip')):neutral(child,pos,rot)
neutral(source,np.zeros(3),np.eye(3),True)
upper_mass=sum(m for m,c in moments);upper_cx=sum(m*c[0] for m,c in moments)/upper_mass
cheek_mass=2*cheek.val().Volume()*1e-9*P['aluminum_density_kg_m3']
pelvis_x=(cheek_mass*.052-upper_mass*upper_cx)/(upper_mass+cheek_mass)
for sign,side in [(1,'left'),(-1,'right')]:add('PART-105_'+side+'_pelvis_cradle',cheek.translate((pelvis_x*1000-52,sign*72.5,hip_z)))
(OUT/'placement.json').write_text(json.dumps({'pelvis_origin_mm':[pelvis_x*1000,0,P['pelvis_z_mm']],'source_rest_upper_mass_with_hands_kg':upper_mass,'source_rest_upper_COM_x_m':upper_cx,'source_hip_frames_mm':[[-52,67.5,-44.045],[-52,-67.5,-44.045]],'cradle_hole_centers_relative_hip_mm':sorted(projected),'source_plate':'ASV1_200_03A','outer_PCD_mm':100.2,'hole_diameter_mm':4.3,'spacer_gap_mm':1,'interface_status':'source nominal projection and MJCF joint frames; physical case-face datum, spacers and longer bolt engagement require fit verification'},indent=2))
for side in [-1,1]:
    # Shaft seat and bolt pattern attach a separately modelled split torque clamp.
    height=2*(z-t/2-30)
    plate=cq.Workplane('XZ').rect(100,height).extrude(4,both=True).translate((0,side*156,30))
    shaft_tool=cq.Workplane('XZ').circle(P['shaft_bore_mm']/2).extrude(200,both=True)
    plate=plate.cut(shaft_tool)
    for x in [-12,12]:plate=plate.cut(cq.Workplane('XZ').center(x,-13).circle(2.75).extrude(200,both=True))
    add('PART-104_'+('left' if side==1 else 'right')+'_axle_seat',plate)
    block=cq.Workplane('XY').box(70,14,60).cut(cq.Workplane('XZ').circle(P['shaft_bore_mm']/2).extrude(20,both=True))
    for x in [-21,21]:block=block.cut(cq.Workplane('XY').center(x,0).circle(3.3).extrude(80,both=True))
    for label,zz in [('lower',-15.375),('cap',15.375)]:
     half=block.intersect(cq.Workplane('XY').box(80,20,29.25).translate((0,0,zz)))
     if label=='lower':
      for x in [-12,12]:half=half.cut(cq.Workplane('XZ').center(x,-13).circle(2.75).extrude(20,both=True))
     add('PART-106_'+('left' if side==1 else 'right')+'_torque_clamp_'+label,half.translate((0,side*167,0)))
    wheel=cq.Workplane('XZ').circle(P['wheel_radius_mm']).extrude(P['wheel_width_mm']/2,both=True).translate((0,side*P['wheel_track_mm']/2,0))
    add('PART-201_'+('left' if side==1 else 'right')+'_wheel_ENVELOPE',wheel,'rubber',P['wheel_mass_kg_each'])
add('PART-202_battery_ENVELOPE',cq.Workplane('XY').box(160,110,60).translate((0,0,50)),'battery',P['battery_mass_kg'])
# Four removable screw-lock parking struts are a separate parked configuration.
for x in [-180,180]:
 for y in [-130,130]:
    foot=cq.Workplane('XY').box(36,36,8).union(cq.Workplane('XY').circle(6).extrude(183))
    foot=foot.union(cq.Workplane('XY').box(100,20,12).translate((-40 if x>0 else 40,0,177))).translate((x,y,-P['wheel_radius_mm']+4))
    add(f'PART-301_parking_{x}_{y}',foot)
assembly=cq.Assembly(name='balancing_body_new_base')
colors={'aluminum':(.18,.25,.30),'rubber':(.055,.065,.075),'battery':(.12,.35,.31),'steel':(.6,.63,.65)}
for name,shape,mat,mass in parts: assembly.add(shape,name=name,color=cq.Color(*colors[mat]))
assembly.save(str(OUT/'new_base.step'))
records=[]
for name,shape,mat,mass in parts:
    s=shape.val();b=Bnd_Box();BRepBndLib.AddOptimal_s(s.wrapped,b,False,False);v=b.Get(); c=s.Center()
    records.append({'id':name,'material':mat,'mass_kg':mass,'mass_evidence':'CAD volume x assumed aluminum density' if mat=='aluminum' else 'provisional allowance','valid':s.isValid(),'solids':len(s.Solids()),'volume_mm3':s.Volume(),'com_mm':[c.x,c.y,c.z],'bbox_mm':[v[3]-v[0],v[4]-v[1],v[5]-v[2]],'bbox_bounds_mm':list(v)})
(OUT/'manifest.json').write_text(json.dumps(records,indent=2))
print(json.dumps({'parts':len(records),'all_valid':all(r['valid'] for r in records),'base_aluminum_kg':sum(r['mass_kg'] for r in records if r['material']=='aluminum')}))
