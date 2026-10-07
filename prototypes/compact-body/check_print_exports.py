"""Independent print-export units/topology/volume checks against the B-rep build manifest."""
from pathlib import Path
import json,hashlib
import numpy as np
import trimesh
ROOT=Path(__file__).resolve().parent
rows=[]
for e in json.loads((ROOT/'checks/build_manifest.json').read_text()):
    f=ROOT/'cad'/(e['name']+'.stl')
    if not f.exists():continue # Purchased standoffs do not have print exports.
    m=trimesh.load(f,force='mesh');b=e['bbox_mm']
    expected=np.array(b[3:])-np.array(b[:3])
    dim_error=float(np.max(np.abs(m.extents-expected)))
    volume_error=abs(abs(m.volume)-e['volume_mm3'])/e['volume_mm3']
    ok=bool(m.is_watertight and m.is_winding_consistent and dim_error<.16 and volume_error<.01 and abs(m.bounds[0,2])<.001)
    rows.append(dict(part=e['name'],status='PASS' if ok else 'FAIL',watertight=bool(m.is_watertight),winding_consistent=bool(m.is_winding_consistent),max_dimension_error_mm=dim_error,relative_volume_error=volume_error,z_min_mm=float(m.bounds[0,2]),sha256=hashlib.sha256(f.read_bytes()).hexdigest()))
(ROOT/'checks/print_export_checks.json').write_text(json.dumps(rows,indent=2)+'\n')
print('Print exports',len(rows),'failures',sum(e['status']=='FAIL' for e in rows),'max dimension error mm',max(e['max_dimension_error_mm'] for e in rows),'max relative volume error',max(e['relative_volume_error'] for e in rows))
if any(e['status']=='FAIL' for e in rows):raise SystemExit(1)
