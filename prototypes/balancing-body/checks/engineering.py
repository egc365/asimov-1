"""Independent dimensional, structural and operating-envelope screening.
No factors here imply that unselected hardware has verified ratings.
"""
import json,pathlib,math,copy,itertools
import numpy as np,cadquery as cq
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
ROOT=pathlib.Path(__file__).resolve().parents[1]
P=json.loads((ROOT/'parameters.json').read_text());M=json.loads((ROOT/'cad/concept/manifest.json').read_text());props=json.loads((ROOT/'checks/mass_properties.json').read_text())
mass=props['total_mass_kg'];g=9.81;body=props['body_mass_kg'];h=props['com_above_axle_m'][2];z=P['wheel_radius_mm']/1000+body*h/mass
# Fresh process STEP round-trip checks every individual part and assembly.
roundtrip=[];shapes={}
for rec in M:
 s=cq.importers.importStep(str(ROOT/'cad/concept'/(rec['id']+'.step'))).val();b=Bnd_Box();BRepBndLib.AddOptimal_s(s.wrapped,b,False,False);v=b.Get();dims=[v[3]-v[0],v[4]-v[1],v[5]-v[2]]
 roundtrip.append({'id':rec['id'],'valid':s.isValid(),'solid_count':len(s.Solids()),'volume_relative_error':abs(s.Volume()/rec['volume_mm3']-1),'bbox_max_error_mm':max(abs(np.array(dims)-rec['bbox_mm']))});shapes[rec['id']]=s
assembly=cq.importers.importStep(str(ROOT/'cad/concept/new_base.step')).val()
critical={
 'deck_bbox_mm':list(M[0]['bbox_mm']),
 'deck_expected_mm':[P['deck_length_mm'],P['deck_width_mm'],P['deck_thickness_mm']],
 'column_expected_mm':[P['column_depth_mm'],P['column_width_mm'],P['column_height_mm']],
 'column_actual_mm':list(M[1]['bbox_mm']),
 'wheel_center_distance_mm':abs(next(r for r in M if r['id']=='PART-201_left_wheel_ENVELOPE')['com_mm'][1]-next(r for r in M if r['id']=='PART-201_right_wheel_ENVELOPE')['com_mm'][1]),
 'shaft_hole_radii_mm':sorted({round(e.radius(),5) for e in shapes['PART-104_left_axle_seat'].Edges() if e.geomType()=='CIRCLE'}),
 'pelvis_underside_to_plate_mm':P['pelvis_z_mm']-.10404499620199203*1000-(P['deck_z_mm']+P['deck_thickness_mm']/2+P['column_height_mm']+6),
 'wheel_to_axle_seat_gap_mm':P['wheel_track_mm']/2-P['wheel_width_mm']/2-(156+4),
 'frame_ground_clearance_mm':P['wheel_radius_mm']+60-(P['deck_z_mm']-P['deck_thickness_mm']/2)}
assert np.allclose(critical['deck_bbox_mm'],critical['deck_expected_mm'],atol=.001)
assert np.allclose(critical['column_actual_mm'],critical['column_expected_mm'],atol=.001)
assert abs(critical['wheel_center_distance_mm']-P['wheel_track_mm'])<.001
assert critical['shaft_hole_radii_mm']==[2.75,P['shaft_bore_mm']/2]
assert abs(critical['pelvis_underside_to_plate_mm']-2)<.001
for rec in roundtrip:assert rec['valid'] and rec['solid_count']==1 and rec['volume_relative_error']<1e-7 and rec['bbox_max_error_mm']<.001
assert len(assembly.Solids())==18 and assembly.isValid()
placement=json.loads((ROOT/'cad/concept/placement.json').read_text());assert len(placement['cradle_hole_centers_relative_hip_mm'])==8
critical['source_cradle_PCD_mm']=placement['outer_PCD_mm'];critical['source_cradle_hole_mm']=placement['hole_diameter_mm'];critical['source_cradle_spacer_gap_mm']=placement['spacer_gap_mm']
(ROOT/'checks/cad_roundtrip.json').write_text(json.dumps({'parts':roundtrip,'assembly_solids':len(assembly.Solids()),'assembly_valid':assembly.isValid(),'critical_dimensions':critical},indent=2))
# Exact BRep intersection for new base only. Deployed parking supports form a separate configuration.
pairs=[]
for a,b in itertools.combinations(M,2):
 if 'parking_' in a['id'] or 'parking_' in b['id']:status='not_run_in_balancing_configuration';vol=None
 else:
  vol=shapes[a['id']].intersect(shapes[b['id']]).Volume();status='overlap' if vol>1e-5 else 'clear_or_touching'
 pairs.append({'a':a['id'],'b':b['id'],'status':status,'intersection_mm3':vol})
(ROOT/'checks/base_pair_matrix.json').write_text(json.dumps(pairs,indent=2))
# Structural screens: 2g vertical load, 0.5g horizontal body load, no FEA.
span=.312;width=.240;thick=P['deck_thickness_mm']/1000;E=69e9;F=2*mass*g;I=width*thick**3/12;moment=F*span/4
deck_stress=moment*(thick/2)/I/1e6;deck_defl=F*span**3/(48*E*I)*1000
outer_d=P['column_depth_mm']/1000;outer_w=P['column_width_mm']/1000;wall=P['column_wall_mm']/1000
Ic=(outer_w*outer_d**3-(outer_w-2*wall)*(outer_d-2*wall)**3)/12
Mc=.5*props['upper_with_hand_allowances_kg']*g*max(0,props['upper_COM_above_axle_m'][2]-P['deck_z_mm']/1000);colstress=Mc*outer_d/2/Ic/1e6
def axle_screen(d_mm):
 d=d_mm/1000;overhang=P['wheel_track_mm']/2000-.156;load=mass*g # 2g vertical, one wheel half total
 bending=32*load*overhang/(math.pi*d**3)/1e6;tau=16*P['torque_limit_Nm_each']/(math.pi*d**3)/1e6;Kt=1.8
 vm=math.sqrt((Kt*bending)**2+3*tau*tau);yield_assumed=250
 return {'diameter_mm':d_mm,'cantilever_m':overhang,'vertical_load_N_each_2g':load,'stress_concentration_assumed':Kt,'bending_MPa_before_Kt':bending,'torsion_MPa':tau,'von_Mises_MPa':vm,'steel_yield_assumed_MPa':yield_assumed,'factor_to_first_yield':yield_assumed/vm,'margin_against_factor_2':yield_assumed/(2*vm),'pass_factor_2':bool(yield_assumed/(2*vm)>=1),'limit':'source shaft material, root radius, flats and torque retention unknown'}
axles=[axle_screen(d) for d in [12,14,16]]
preload=7000;mu=.15;shaft_r=P['shaft_diameter_mm']/2000;clamp_capacity=2*preload*mu*shaft_r;cap_thick=.030-P['shaft_bore_mm']/2000
cap_stress=6*preload*(.021-P['shaft_bore_mm']/2000)/(.014*cap_thick**2)/1e6
results={
 'CALC-001_mass':props,
 'CALC-002_tip':{'total_COM_height_m':z,'lateral_static_tip_deg':math.degrees(math.atan(.2/z)),'parking_fore_aft_static_tip_deg':math.degrees(math.atan(.18/z)),'lateral_accel_static_tip_m_s2':g*.2/z,'two_wheel_poweroff_fore_aft_support_margin_m':0,'limits':'level floor, no slip, fixed posture; roll is not corrected by fore/aft balance controller'},
 'CALC-003_deck':{'span_m':span,'vertical_case_g':2,'force_N':F,'bending_stress_MPa':deck_stress,'deflection_mm':deck_defl,'allowable_unwelded_assumed_MPa':120,'margin_unwelded':120/deck_stress,'weld_zone_yield_assumed_MPa':80,'weld_and_hole_concentration_assumed':1.5,'welded_margin_factor_2':40/(1.5*deck_stress),'six_mm_deck_welded_margin':40/(1.5*deck_stress*(P['deck_thickness_mm']/6)**2),'limits':'simply supported one-dimensional section; assumed weld-zone strength and stress concentration require coupon/connection proof; no fatigue or FEA'},
 'CALC-004_column':{'horizontal_case_g':.5,'moment_Nm':Mc,'bending_stress_MPa':colstress,'margin':120/colstress,'limits':'prismatic tube only; welds and bolts not included'},
 'CALC-005_axle':axles,
 'CALC-006_tolerance':{'shaft_nominal_mm':P['shaft_diameter_mm'],'shaft_tolerance_mm':.02,'bore_nominal_mm':P['shaft_bore_mm'],'bore_tolerance_mm':.05,'diametral_clearance_min_mm':P['shaft_bore_mm']-.05-P['shaft_diameter_mm']-.02,'diametral_clearance_max_mm':P['shaft_bore_mm']+.05-P['shaft_diameter_mm']+.02,'M6_clearance_proposed_mm':6.6,'clamp_split_gap_mm':1.5,'source_cradle_nominal_holes_mm':4.3,'source_cradle_nominal_PCD_mm':100.2,'limits':'new machining tolerances proposed, source mounting holes measured nominal only; source case datum, screw engagement and clamp friction require physical fit/proof'},
 'CALC-007_transport':{'wheel_rpm_at_0_4_m_s':.4/(2*math.pi*P['wheel_radius_mm']/1000)*60,'mechanical_power_W_each_at_6Nm_0_4m_s':6*.4/(P['wheel_radius_mm']/1000),'UART_command_time_ms_115200':8*10/115200*1000,'UART_feedback_18bytes_ms_115200':18*10/115200*1000,'UART_feedback_22bytes_ms_115200':22*10/115200*1000,'firmware_filter_63_percent_time_ms':-5/math.log(1-.1),'minimum_effective_Kt_Nm_per_A_for_6Nm_at_15A':6/15,'copper_heat_formula':'P_copper=I_rms^2 R; actual motor R and Kt unavailable','limits':'serial time excludes scheduling; nominal filter alone adds about 47ms response time; no electrical efficiency or thermal rating implied'},
 'CALC-008_clamp':{'M6_bolts_per_clamp':2,'preload_assumed_N_each':preload,'friction_assumed':mu,'shaft_radius_m':shaft_r,'torque_capacity_Nm':clamp_capacity,'capacity_over_6Nm':clamp_capacity/6,'friction_for_factor_2_min':12/(2*preload*shaft_r),'cap_bending_stress_MPa':cap_stress,'cap_margin_assumed':120/cap_stress,'bolt_M6_area_assumed_mm2':20.1,'bolt_8_8_proof_assumed_MPa':580,'preload_fraction_of_proof':preload/(20.1*580),'limits':'friction capacity is not validated retention; smooth shaft only, oily/flat shafts require different insert or keyed torque arm. No wrench setting authorized by this calculation'},
 'CALC-009_cradle':{'source_hole_count_each':8,'source_PCD_mm':100.2,'vertical_upper_load_N_2g':2*props['upper_with_hand_allowances_kg']*g,'nominal_direct_shear_N_each_M4':2*props['upper_with_hand_allowances_kg']*g/16,'horizontal_upper_moment_Nm_0_5g':.5*props['upper_with_hand_allowances_kg']*g*(props['upper_COM_above_axle_m'][2]-(P['pelvis_z_mm']/1000-.044045)),'limits':'source hole nominal pattern, new cradle projected to source hip joint frame; source case-face datum, thread engagement, spacer stack and structural case capacity unverified. Source outer fasteners must carry loads through metal, not housing shell'}
}
(ROOT/'checks/engineering_results.json').write_text(json.dumps(results,indent=2))
print(json.dumps({'STEP_valid_parts':len(roundtrip),'assembly_solids':len(assembly.Solids()),'critical_dimensions':critical,'structure':results['CALC-003_deck'],'axle_screens':axles,'tip':results['CALC-002_tip'],'overlap_pairs':[x for x in pairs if x['status']=='overlap']},indent=2))
