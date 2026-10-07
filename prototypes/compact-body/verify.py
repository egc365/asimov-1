"""Fresh-process CAD, source interface, mesh motion and arithmetic screening.
Results are evidence of these particular checks, not fabrication or safety certification.
"""
from pathlib import Path
import json, itertools, math, hashlib, xml.etree.ElementTree as ET
import numpy as np
import cadquery as cq
import trimesh
from scipy.spatial.transform import Rotation
from design import build, DEFAULTS, BASE_HOLES, ARM_HOLES

ROOT=Path(__file__).resolve().parent
WORK=ROOT.parents[2]
checks=[]
def record(name,status,detail): checks.append(dict(check=name,status=status,detail=detail)); print(name,status,flush=True)
def metrics(s):
    b=s.BoundingBox()
    return dict(solids=len(s.Solids()),valid=s.isValid(),volume=s.Volume(),bbox=[b.xmin,b.ymin,b.zmin,b.xmax,b.ymax,b.zmax])

def origin(e,mm=True):
    a=np.eye(4)
    if e is None:return a
    a[:3,3]=np.fromstring(e.get('xyz','0 0 0'),sep=' ')*(1000 if mm else 1)
    a[:3,:3]=Rotation.from_euler('xyz',np.fromstring(e.get('rpy','0 0 0'),sep=' ')).as_matrix()
    return a

class URDF:
    def __init__(self,path,meshes_mm=False):
        self.path=Path(path);self.root=ET.parse(path).getroot()
        self.links={e.get('name'):e for e in self.root.findall('link')}
        self.joints=self.root.findall('joint')
        children={j.find('child').get('link') for j in self.joints}
        self.base=next(n for n in self.links if n not in children)
        self.meshes=[]
        for name,link in self.links.items():
            for i,v in enumerate(link.findall('visual')):
                e=v.find('geometry/mesh')
                if e is None:continue
                m=trimesh.load(self.path.parent/e.get('filename'),force='mesh')
                scale=np.fromstring(e.get('scale','1 1 1'),sep=' ')
                # LeKiwi files are mm with explicit 0.001 URDF scale; SO101 meshes are metres.
                m.apply_scale(scale*1000)
                m.apply_transform(origin(v.find('origin')))
                self.meshes.append((name,i,m))
        self.limits={j.get('name'):(float(j.find('limit').get('lower')),float(j.find('limit').get('upper'))) for j in self.joints if j.find('limit') is not None}
    def fk(self,q):
        out={self.base:np.eye(4)};pending=list(self.joints)
        while pending:
            old=len(pending)
            for j in pending[:]:
                parent=j.find('parent').get('link');child=j.find('child').get('link')
                if parent not in out:continue
                t=origin(j.find('origin'));ang=q.get(j.get('name'),0)
                if j.get('type') in ['revolute','continuous']:
                    axis=np.fromstring(j.find('axis').get('xyz'),sep=' ');r=np.eye(4);r[:3,:3]=Rotation.from_rotvec(axis*ang).as_matrix();t=t@r
                out[child]=out[parent]@t;pending.remove(j)
            if len(pending)==old:raise ValueError('Disconnected URDF')
        return out
    def scene(self,q,root=np.eye(4),skip=()):
        fk=self.fk(q);out=[]
        for link,i,m in self.meshes:
            if link in skip:continue
            n=m.copy();n.apply_transform(root@fk[link]);out.append((f'{link}_{i}',n))
        return out

def arm_root(x,p=DEFAULTS):
    t=np.eye(4);t[:3,:3]=Rotation.from_euler('z',90,degrees=True).as_matrix()
    t[:3,3]=[x,p['arm_y']-10.71315,p['deck_bottom']+p['deck_thickness']+2.40026]
    return t

def main():
    p,parts=build(json.loads((ROOT/'parameters.json').read_text()));manifest=json.loads((ROOT/'checks/build_manifest.json').read_text())
    native=cq.Assembly.load(str(ROOT/'cad/compact_body.xbf'),'XBF').toCompound()
    neutral=cq.importers.importStep(str(ROOT/'cad/compact_body.step')).val()
    n=metrics(native);s=metrics(neutral)
    expected=sum(e['volume_mm3'] for e in manifest)
    for label,m in [('native',n),('STEP',s)]:
        passed=m['valid'] and m['solids']==len(parts) and abs(m['volume']-expected)/expected<1e-6
        record(f'reopen_{label}', 'PASS' if passed else 'FAIL',m)
    record('native_STEP_bbox','PASS' if np.max(np.abs(np.array(n['bbox'])-s['bbox']))<.001 else 'FAIL',dict(max_error_mm=float(np.max(np.abs(np.array(n['bbox'])-s['bbox'])))))
    allparts=[]
    for name,shape in parts.items():
        imported=cq.importers.importStep(str(ROOT/'cad'/f'{name}.step')).val()
        m=metrics(imported);allparts.append(dict(name=name,**m))
        assert m['valid'] and m['solids']==1 and m['volume']>0,name
    record('individual_STEP_solids','PASS',allparts)
    try:build({'wall':1.0})
    except ValueError as e:record('negative_parameter_wall','PASS',str(e))
    else:record('negative_parameter_wall','FAIL','Thin wall accepted')
    # Independent measurement of bores by line/cylinder intersection, not just parameter echo.
    deck=cq.importers.importStep(str(ROOT/'cad/deck.step')).val()
    diameter_tests=[]
    for x,y in BASE_HOLES:
        probe=cq.Solid.makeCylinder(1.79,10,cq.Vector(x,y,69))
        outside=cq.Solid.makeCylinder(1.81,8,cq.Vector(x,y,70))
        diameter_tests.append(dict(x=x,y=y,inner_intersection_mm3=deck.intersect(probe).Volume(),outer_intersection_mm3=deck.intersect(outside).Volume()))
    record('M3_clearance_independent','PASS' if all(e['inner_intersection_mm3']<1e-6 and e['outer_intersection_mm3']>.01 for e in diameter_tests) else 'FAIL',diameter_tests)
    source_mesh=trimesh.load(WORK/'source-lekiwi/3DPrintMeshes/base_plate_layer2.stl',force='mesh')
    holedata=json.loads((ROOT/'source/lekiwi_top_hole_measurements.json').read_text())
    match=[]
    for x,y in BASE_HOLES:
        e=min(holedata,key=lambda e:math.hypot(e[0]-x,e[1]-y))
        match.append(dict(target=[x,y],source_circle=e,axis_error_mm=math.hypot(e[0]-x,e[1]-y)))
    record('LeKiwi_existing_mount_axes','PASS' if max(e['axis_error_mm'] for e in match)<.01 else 'FAIL',match)
    intersections=[]
    for (a,sa),(b,sb) in itertools.combinations(parts.items(),2):
        # Shared faces/contact are allowed; any nonzero intersection is reported.
        v=sa.intersect(sb).Volume()
        if v>1e-5:intersections.append(dict(a=a,b=b,volume_mm3=v))
    record('body_pairwise_interference','PASS' if not intersections else 'FAIL',intersections)
    source_arm_path=WORK/'source-soarm/STEP/SO101/SO101 Assembly.step'
    prior_file=ROOT/'checks/results.json';hash_file=ROOT/'checks/source_hashes.json'
    cached=None
    if prior_file.exists() and hash_file.exists():
        oldhashes=json.loads(hash_file.read_text())
        known=next((h['sha256'] for h in oldhashes if h['path'].endswith('SO101 Assembly.step')),None)
        if known==hashlib.sha256(source_arm_path.read_bytes()).hexdigest():
            cached=next((e for e in json.loads(prior_file.read_text()) if e['check']=='source_SO101_full_STEP'),None)
    if cached:
        record('source_SO101_full_STEP',cached['status'],{**cached['detail'],'provenance':'Reused actual baseline geometry check after exact SHA256 match; original full STEP was imported and tested on first run.'})
    else:
        source_arm=cq.importers.importStep(str(source_arm_path)).val()
        m=metrics(source_arm);record('source_SO101_full_STEP','FAIL' if not m['valid'] else 'PASS',m)
    fit=dict(M3_nominal_screw_max_mm=3.0,printed_hole_nominal_mm=3.6,assumed_hole_error_mm=.2,
             min_diametral_clearance_mm=3.6-.2-3.0,face_edge_nominal_gap_mm=.6,
             assumed_each_edge_error_mm=.2,face_edge_worst_gap_mm=.6-.4,
             status='Positive clearance under assumed errors; printer errors require a measured coupon')
    record('tolerance_stack','CONDITIONAL',fit)
    # Actual mesh collision checks at reproducible samples of all six source joint coordinates.
    arm=URDF(WORK/'source-soarm/Simulation/SO101/so101_new_calib.urdf')
    bm=trimesh.collision.CollisionManager()
    for name,shape in parts.items():
        if 'SO101_base' in name:continue # mating source root, excluded intentionally
        vs,fs=shape.tessellate(.25)
        bm.add_object(name,trimesh.Trimesh(vertices=[v.toTuple() for v in vs],faces=fs,process=False))
    rng=np.random.default_rng(20261006)
    poses=[{j:0. for j in arm.limits}]
    poses += [{j:float(rng.uniform(lo,hi)) for j,(lo,hi) in arm.limits.items()} for _ in range(48)]
    # A narrow candidate commissioning range, sampled at each joint's +/-5 degree boundary.
    # Passing these points does not approve paths between them or combined joint extremes.
    for joint in arm.limits:
        for sign in [-1,1]:
            poses.append({j:(sign*math.radians(5) if j==joint else 0.) for j in arm.limits})
    samples=[]
    # Retain collision BVHs and move them by the URDF rigid transforms.
    # This checks the same source triangles without rebuilding every BVH at every pose.
    managers=[]
    moving=[(link,f'{link}_{i}',m) for link,i,m in arm.meshes if link!='base_link']
    for x in [-p['arm_x'],p['arm_x']]:
        cm=trimesh.collision.CollisionManager()
        for link,name,m in moving:cm.add_object(name,m)
        managers.append(cm)
    for i,q in enumerate(poses):
        hits=[];fk=arm.fk(q)
        for cm,side,x in zip(managers,['left','right'],[-p['arm_x'],p['arm_x']]):
            for link,name,m in moving:cm.set_transform(name,arm_root(x,p)@fk[link])
            hit,pairs=cm.in_collision_other(bm,return_names=True)
            if hit:hits += [f'{side}:{a}/{b}' for a,b in sorted(pairs)]
        hit,pairs=managers[0].in_collision_other(managers[1],return_names=True)
        if hit:hits += [f'left:{a}/right:{b}' for a,b in sorted(pairs)]
        samples.append(dict(sample=i,joints_rad=q,collisions=hits,status='COLLISION' if hits else 'CLEAR_AT_THIS_SAMPLE'))
    (ROOT/'checks/motion_samples.json').write_text(json.dumps(samples,indent=2)+'\n')
    collisions=sum(bool(s['collisions']) for s in samples)
    record('sampled_source_URDF_motion','LIMITED' if collisions else 'PASS_SAMPLED_ONLY',dict(samples=len(samples),collision_samples=collisions,neutral_sample=samples[0],small_single_joint_sweep_collision_samples=sum(bool(s['collisions']) for s in samples[49:]),not_checked=['continuous trajectories','combined commissioning joint extremes','independent left/right joint combinations','arm self collision','cables','servo calibration','dynamic control'],source_frame_alignment='base bounding box matched after +90 deg Z rotation; source URDF/STEP revisions differ'))
    # Transparent screening assumptions; stall torque is not a payload rating.
    custom_vol=sum(s.Volume() for name,s in parts.items() if 'SO101_base' not in name and not name.startswith('standoff'))
    payload=.100;arm_mass=.632006;reach=.30;g=9.81
    torque=(payload*reach+arm_mass*.15)*g
    load=arm_mass*g;span=p['arm_x']-40;width=87.;thickness=p['deck_thickness'];E=1500.
    stress=6*load*span/(width*thickness**2)
    deflection=load*span**3/(3*E*(width*thickness**3/12))
    power=dict(servo_count=15,conservative_7_4V_stall_current_A_each=2.5,all_stall_A_screen=37.5,three_assumed_5A_supplies_A=15,per_arm_stall_A_screen=15,per_arm_assumed_supply_A=5,result='FAIL conservative simultaneous stall screen. Current is a 7.4V variant reference, not a measured 5V value. Source listing rail ratings, adapter current capability and continuous duty unverified.')
    record('power_screen','FAIL',power)
    record('load_screen','CONDITIONAL',dict(payload_assumption_kg=payload,arm_source_URDF_mass_kg=arm_mass,reach_assumption_m=reach,shoulder_static_torque_Nm=torque,repo_6V_stall_torque_Nm=16.5*.0980665,torque_ratio_to_6V_stall=torque/(16.5*.0980665),custom_solid_PLA_mass_upper_bound_kg=custom_vol*1.24e-6,deck_beam_stress_MPa=stress,deck_beam_deflection_mm=deflection,assumed_print_modulus_MPa=E,conclusion='No qualified payload at 5V: continuous servo rating unavailable; simple beam screen excludes creep, notch, layer strength and joint stiffness'))
    # Wheel-contact support triangle; masses are explicit screening assumptions.
    tri=np.array([[.00061,-.126375],[.092511,.054116],[-.092428,.052659]])
    support=[]
    for k in range(3):
        a,b=tri[k],tri[(k+1)%3];d=b-a;support.append(abs(d[0]*(-a[1])-d[1]*(-a[0]))/np.linalg.norm(d))
    support_min=float(min(support));base_mass_assumed=1.0;body_mass=custom_vol*1.24e-6
    total=base_mass_assumed+body_mass+2*arm_mass+2*payload
    shell_y_moment=sum(s.Volume()*1.24e-6*s.Center().y/1000 for name,s in parts.items() if 'SO101_base' not in name and not name.startswith('standoff'))
    com_y=(shell_y_moment-2*arm_mass*.15-2*payload*.30)/total
    point=np.array([0,com_y]);margins=[]
    for k in range(3):
        a,b=tri[k],tri[(k+1)%3];d=b-a;v=point-a;margins.append(float((d[0]*v[1]-d[1]*v[0])/np.linalg.norm(d)))
    margin=min(margins)
    record('tipping_screen','CONDITIONAL',dict(contact_xy_m=tri.tolist(),center_to_closest_edge_m=support_min,base_mass_assumed_kg=base_mass_assumed,solid_shell_mass_upper_bound_kg=body_mass,total_screen_mass_kg=total,two_arms_extended_COM_y_m=com_y,signed_COM_edge_margins_m=margins,minimum_static_margin_m=margin,assumed_COM_height_m=.22,lateral_acceleration_threshold_screen_m_s2=max(0,margin)*9.81/.22,warning='Wheel centers approximate contacts; assumed base mass and solid shell density. Measure COM/contact patch. Dynamic braking, external loads, slopes and reduced infill are unvalidated.'))
    # Source checksums make the retained baselines inspectable.
    rows=[]
    for path in [WORK/'source-soarm/STEP/SO101/Base_SO101.step',WORK/'source-soarm/STEP/SO101/SO101 Assembly.step',WORK/'source-soarm/Simulation/SO101/so101_new_calib.urdf',WORK/'source-lekiwi/3DPrintMeshes/base_plate_layer2.stl',WORK/'source-lekiwi/URDF/LeKiwi.urdf',ROOT/'source/SO101_base_assembly_frame.step']:
        rows.append(dict(path=str(path.relative_to(WORK)),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),bytes=path.stat().st_size))
    (ROOT/'checks/source_hashes.json').write_text(json.dumps(rows,indent=2)+'\n')
    (ROOT/'checks/results.json').write_text(json.dumps(checks,indent=2)+'\n')

if __name__=='__main__':main()
