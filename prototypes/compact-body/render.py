"""Engineering views from actual CAD and pinned URDF meshes, never generative imagery."""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import trimesh
from design import build
from verify import ROOT,WORK,URDF,arm_root

p,parts=build(json.loads((ROOT/'parameters.json').read_text()));scene=[]
for name,s in parts.items():
    vertices,faces=s.tessellate(.4)
    m=trimesh.Trimesh(vertices=[v.toTuple() for v in vertices],faces=faces,process=False)
    color='#426652' if 'column' in name or name=='head_shell' else '#ded9c9'
    if name=='face_panel':color='#293336'
    scene.append((name,m,color))
arm=URDF(WORK/'source-soarm/Simulation/SO101/so101_new_calib.urdf')
q={j:0 for j in arm.limits}
for side,x in [('L',-p['arm_x']),('R',p['arm_x'])]:
    for name,m in arm.scene(q,arm_root(x,p),skip=['base_link']):
        scene.append((side+'_'+name,m,'#777d82' if 'sts' in name.lower() else '#ded9c9'))
base=URDF(WORK/'source-lekiwi/URDF/LeKiwi.urdf')
keep=[x for x in base.links if x.startswith(('base_plate','drive_motor','ST3215','4-Omni','omni_wheel','servo_controller','94868'))]
for name,m in base.scene({},skip=[x for x in base.links if x not in keep]):
    scene.append(('base_'+name,m,'#374e42' if 'plate' in name else '#747c82'))
g=trimesh.Scene()
for name,m,color in scene:
    m.visual.face_colors=matplotlib.colors.to_rgba_array(color)[0]*255
    g.add_geometry(m,node_name=name)
(ROOT/'cad/robot_reference_assembly.glb').write_bytes(g.export(file_type='glb'))
bounds=np.vstack([m.bounds for _,m,_ in scene]);lo=bounds.min(0);hi=bounds.max(0)
fig=plt.figure(figsize=(14,6),facecolor='#e4e7e6')
for idx,(title,elev,azim) in enumerate([('ASSEMBLY / source mechanisms',23,-65),('FRONT / camera faces -Y',0,-90),('SIDE / reach is source URDF',0,0)]):
    ax=fig.add_subplot(1,3,idx+1,projection='3d')
    ax.set_facecolor('#e4e7e6')
    for name,m,c in scene:
        # Full reference is in GLB; preview is a continuous decimated surface, not dropped triangles.
        preview=m.simplify_quadric_decimation(face_count=10000) if len(m.faces)>10000 else m
        ax.add_collection3d(Poly3DCollection(preview.vertices[preview.faces],facecolors=c,edgecolors=c,linewidths=0,alpha=1,shade=True,rasterized=True))
    ax.set_xlim(lo[0]-10,hi[0]+10);ax.set_ylim(lo[1]-10,hi[1]+10);ax.set_zlim(lo[2]-10,hi[2]+10)
    ax.set_box_aspect(hi-lo);ax.view_init(elev=elev,azim=azim);ax.set_axis_off();ax.set_title(title,fontsize=10,pad=10)
fig.suptitle('COMPACT BODY • CAD PROTOTYPE • millimetres',fontsize=19,y=.94)
fig.text(.04,.08,'340 mm deck • 124 × 84 × 80 mm fixed head • 2 × SO101 followers • original 3-wheel LeKiwi base',fontsize=12)
fig.text(.04,.045,'New body geometry + source URDF reference meshes. Neutral pose shown; unrestricted arm motion has not been approved.',fontsize=10)
fig.savefig(ROOT/'drawings/assembly_views.png',dpi=160,bbox_inches='tight');plt.close(fig)
(ROOT/'checks/reference_scene_bounds.json').write_text(json.dumps(dict(min_mm=lo.tolist(),max_mm=hi.tolist(),height_mm=float(hi[2]-lo[2])),indent=2)+'\n')
print('Saved engineering views and reference GLB',hi-lo,flush=True)
