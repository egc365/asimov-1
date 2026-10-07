"""Source-linked assembly BOM plus fabricated parts and unresolved procurement requirements."""
import pathlib,json,csv,xml.etree.ElementTree as ET,subprocess
ROOT=pathlib.Path(__file__).resolve().parents[1];REPO=ROOT.parents[1]
src=ET.parse(ROOT/'source/asimov_1.xml').getroot();upper=src.find('./worldbody/body')
for b in list(upper):
 if b.tag=='body' and b.get('name').startswith(('left_hip','right_hip')):upper.remove(b)
meshes={e.get('name'):e.get('file') for e in src.findall('./asset/mesh')};rows=[]
for i,b in enumerate(upper.iter('body'),1):
 geom=b.find("geom[@class='visual']");mass=float(b.find('inertial').get('mass'))
 rows.append({'id':f'PART-SRC-{i:03}','part':b.get('name'),'quantity':1,'mass_kg':mass,'evidence':'pinned source MJCF assembly inertia','path':'sim-model/assets/meshes/'+meshes[geom.get('mesh')],'status':'source assembly retained; procurement SKU and fastener contents not reconstructed from STL','price_USD':'unquoted'})
for r in json.loads((ROOT/'cad/concept/manifest.json').read_text()):
 rows.append({'id':r['id'],'part':r['id'].split('_',1)[1],'quantity':1,'mass_kg':r['mass_kg'],'evidence':r['mass_evidence'],'path':'prototypes/balancing-body/cad/concept/'+r['id']+'.step','status':'purchased motor/battery envelope: SKU unresolved' if 'ENVELOPE' in r['id'] else 'new fabrication geometry; connections provisional','price_USD':'unquoted'})
for pid,part,mass,requirement in [('PART-401','Two hands / end effectors',1,'combined allowance only; select source hand CAD, verify wrist interface and actual mass'),('PART-402','Balance MCU, IMU and motor-control electronics',.6,'candidate must support fresh IMU samples and deterministic <=5ms local loop; source hoverboard FOC board compatibility unverified'),('PART-403','Fasteners, power cables, fuse, DC/DC and harness',.35,'allowance only; connector and electrical protection selection unresolved')]:
 rows.append({'id':pid,'part':part,'quantity':1,'mass_kg':mass,'evidence':'provisional mass budget','path':'parameters.json','status':requirement,'price_USD':'unquoted'})
with (ROOT/'parts_list.csv').open('w') as f:
 w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
assert abs(sum(r['mass_kg'] for r in rows)-json.loads((ROOT/'checks/mass_properties.json').read_text())['total_mass_kg'])<1e-9
if (REPO/'.git').exists():
 paths=subprocess.check_output(['git','ls-tree','-r','--name-only','35ae7b3581ce36762bfddb00b8fc5e7c957ee7fa','mechanical'],cwd=REPO,text=True).splitlines()
 inventory=[p for p in paths if '/FABRICATION/' in p and p.lower().endswith('.step')]
else:
 # Standalone review ZIP carries the immutable inventory without a .git directory.
 cached=json.loads((ROOT/'source/mechanical_inventory.json').read_text())
 assert cached['revision']=='35ae7b3581ce36762bfddb00b8fc5e7c957ee7fa'
 inventory=cached['paths']
(ROOT/'source/mechanical_inventory.json').write_text(json.dumps({'revision':'35ae7b3581ce36762bfddb00b8fc5e7c957ee7fa','inventory_only_not_assembly_quantities':True,'paths':inventory},indent=2))
print(json.dumps({'BOM_rows':len(rows),'mass_sum_kg':sum(r['mass_kg'] for r in rows),'source_fabrication_files':len(inventory),'priced_SKUs':0,'release':'review BOM, not a complete purchasable functional parts list'}))
