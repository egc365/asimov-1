"""Finite pairwise PHYSICAL test plan; generated rows are not completed tests."""
import itertools,json,pathlib
ROOT=pathlib.Path(__file__).resolve().parents[1]
factors={'wheel_torque_Nm_each':[3,6],'local_delay_ms':[5,20,40],
         'measured_floor_friction':[0.2,0.6],'upper_pose':['rest','slow_reach'],
         'power':['on','cut_on_catch_rig']}
names=list(factors);allrows=list(itertools.product(*factors.values()))
def pairs(row):return {(i,row[i],j,row[j]) for i,j in itertools.combinations(range(len(names)),2)}
required=set().union(*(pairs(row) for row in allrows));remaining=set(required);selected=[]
while remaining:
 row=max(allrows,key=lambda row:len(pairs(row)&remaining))
 selected.append(row);remaining-=pairs(row);allrows.remove(row)
covered=set().union(*(pairs(row) for row in selected))
assert covered==required and len(selected)==len(set(selected))
rows=[]
for i,row in enumerate(selected,1):
 rows.append({'test_id':f'TEST-PHY-{i:03}','factors':dict(zip(names,row)),
 'status':'not run','evidence_type':'planned physical test',
 'mechanism':'coupled traction, latency, arm COM and power-loss instability',
 'acceptance':'guarded specimen; on-power pitch within 1 deg and rate below .02 rad/s at 8 s, no structural movement; power-cut load caught without contact outside rig',
 'prerequisites':'identified motor, verified mount and voltage ratings, load-rated catch rig, logged local IMU and encoder timestamps'})
result={'factor_levels':factors,'full_factorial_size':48,'excluded_pairs':[],
 'required_pairs':len(required),'covered_pairs':len(covered),'missing_pairs':[],
 'duplicate_rows':0,'plan_rows':rows,
 'limits':'Declared-factor pair coverage only; no physical rows run, no reliability or higher-order completeness claim. Power-cut tests require independent mechanical catch.'}
(ROOT/'checks/physical_pairwise_plan.json').write_text(json.dumps(result,indent=2))
print(json.dumps({'planned_rows':len(rows),'covered_pairs':len(covered),'physical_tests_run':0}))
