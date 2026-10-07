"""Compact robot body: editable CadQuery 2.7 model, dimensions in millimetres.
Run from this directory. Reference checkouts are siblings of source-asimov.
No actuator housing, gear or gripper is invented here; source mechanisms are retained.
"""
from pathlib import Path
import json, math
import cadquery as cq
import trimesh

ROOT = Path(__file__).resolve().parent
DEFAULTS = dict(deck_diameter=340.0, deck_thickness=8.0, deck_bottom=70.0,
                arm_x=105.0, arm_y=-20.0, hole_diameter=3.6,
                column_width=64.0, column_depth=40.0, column_y=55.0,
                column_height=192.0, wall=3.0, head_width=124.0,
                head_depth=84.0, head_height=80.0, camera_aperture=24.0)
ARM_HOLES = [(-35.,-20.),(35.,-20.),(-50.,20.),(50.,20.)]
BASE_HOLES = [(-40.,-40.),(40.,-40.),(-40.,40.),(40.,40.)]
COLUMN_HOLES = [(-35.,35.),(35.,35.),(-35.,75.),(35.,75.)]

def box(w,d,h,x=0,y=0,z=0):
    return cq.Workplane('XY').box(w,d,h,centered=(True,True,False)).translate((x,y,z)).val()

def bores(s, points, radius, z, height):
    for x,y in points:
        s=s.cut(cq.Solid.makeCylinder(radius,height,cq.Vector(x,y,z)))
    return s

def build(p=None):
    p={**DEFAULTS,**(p or {})}
    if not 2.4 <= p['wall'] <= 5 or p['hole_diameter'] < 3.4:
        raise ValueError('wall >=2.4 mm and M3 clearance >=3.4 mm required')
    if p['deck_diameter'] > 340 or p['deck_diameter'] < 335:
        raise ValueError('deck must retain arm flange and fit 350 mm bed with margin')
    if p['arm_x'] < 104:
        raise ValueError('arm roots too close to central column')
    t=p['deck_thickness']; z=p['deck_bottom']; top=z+t
    parts={}
    deck=cq.Solid.makeCylinder(p['deck_diameter']/2,t,cq.Vector(0,0,z))
    inner_holes=[(-20,45),(20,45),(-20,65),(20,65)]
    holes=BASE_HOLES+COLUMN_HOLES+inner_holes
    for side in [-1,1]:
        holes += [(side*p['arm_x']+x,p['arm_y']+y) for x,y in ARM_HOLES]
    deck=bores(deck,holes,p['hole_diameter']/2,z-1,t+2)
    # Central cable aperture; USB plugs pass through before the rear cover is fitted.
    deck=deck.cut(box(28,18,t+2,0,55,z-1))
    parts['deck']=deck
    for i,(x,y) in enumerate(BASE_HOLES):
        # Purchased 20 mm M3 female/female standoff envelope, not a threaded model.
        parts[f'standoff_{i+1}']=bores(cq.Solid.makeCylinder(3,20,cq.Vector(x,y,50)),[(x,y)],1.5,49,22)
    flange=box(90,70,8,0,55,top)
    flange=bores(flange,COLUMN_HOLES,p['hole_diameter']/2,top-1,10)
    flange=bores(flange,inner_holes,1.8,top-1,10)
    flange=flange.cut(box(28,18,10,0,55,top-1))
    parts['column_foot']=flange
    cz=top+8; h=p['column_height']; w=p['column_width']; d=p['column_depth']; wall=p['wall']
    column=box(w,d,h,0,p['column_y'],cz).cut(box(w-2*wall,d-2*wall,h+2,0,p['column_y'],cz-1))
    # Open rear, removable cover retained by two external hook-and-loop straps.
    column=column.cut(box(w-2*wall,wall+2,h-12,0,p['column_y']+d/2-wall/2,cz+6))
    for zz in [cz+3,cz+h-3]:
        for yy in [45,65]:
            column=column.cut(cq.Solid.makeCylinder(1.8,w+2,cq.Vector(-w/2-1,yy,zz),cq.Vector(1,0,0)))
    parts['column']=column
    parts['column_rear_cover']=box(w-2*wall,wall,h-12,0,p['column_y']+d/2+wall/2,cz+6)
    # Crossbars bridge the tube at both ends. M3 through bolts, no assumed printed threads.
    for name,zz in [('column_lower_bridge',cz),('column_upper_bridge',cz+h-6)]:
        bridge=box(w-2*wall,d-2*wall,6,0,p['column_y'],zz)
        bridge=bores(bridge,[(-20,45),(20,45),(-20,65),(20,65)],1.8,zz-1,8)
        bridge=bridge.cut(box(20,12,8,0,55,zz-1))
        for yy in [45,65]:
            bridge=bridge.cut(cq.Solid.makeCylinder(1.8,w+2,cq.Vector(-w/2-1,yy,zz+3),cq.Vector(1,0,0)))
        parts[name]=bridge
    hz=cz+h; hw=p['head_width']; hd=p['head_depth']; hh=p['head_height']; hy=45
    shell=box(hw,hd,hh,0,hy,hz).cut(box(hw-2*wall,hd-wall+2,hh-2*wall,0,hy-wall/2-1,hz+wall))
    shell=bores(shell,[(-20,45),(20,45),(-20,65),(20,65)],1.8,hz-1,wall+2)
    shell=shell.cut(box(20,12,wall+2,0,55,hz-1))
    face_holes=[(x,hz+zz) for x in [-56,56] for zz in [10,70]]
    for x,zz in face_holes:
        boss=box(8,8,8,math.copysign(57,x),10,zz-4)
        shell=shell.fuse(boss)
    for x,zz in face_holes:
        shell=shell.cut(cq.Solid.makeCylinder(1.8,16,cq.Vector(x,1,zz),cq.Vector(0,1,0)))
    camera_holes=[(-15,20),(15,20),(-15,34),(15,34)]
    shell=bores(shell,camera_holes,1.8,hz-1,5)
    parts['head_shell']=shell
    # Face occupies the opening; clearances are intentional and measured in verify.py.
    face=box(hw-2*wall-1.2,wall,hh-2*wall-1.2,0,hy-hd/2+wall/2,hz+wall+.6)
    for x,zz,r in [(-22,hz+56,5),(22,hz+56,5),(0,hz+24,p['camera_aperture']/2)]:
        face=face.cut(cq.Solid.makeCylinder(r,8,cq.Vector(x,hy-hd/2-2,zz),cq.Vector(0,1,0)))
    for x,zz in face_holes:
        face=face.cut(cq.Solid.makeCylinder(1.8,8,cq.Vector(x,1,zz),cq.Vector(0,1,0)))
    parts['face_panel']=face
    # Camera shelf is universal: two slots accept a Velcro strap; no unverified PCB hole pattern.
    shelf=box(54,40,3,0,27,hz+8)
    for x in [-23,23]: shelf=shelf.cut(box(3,28,5,x,27,hz+7))
    shelf=bores(shelf,camera_holes,1.8,hz+7,5)
    parts['camera_shelf']=shelf
    for i,(x,y) in enumerate(camera_holes):
        parts[f'camera_spacer_{i+1}']=bores(cq.Solid.makeCylinder(3.5,5,cq.Vector(x,y,hz+3)),[(x,y)],1.8,hz+2,7)
    # Keep the exact source base, changing only four mounting bores in existing flange material.
    src=cq.importers.importStep(str(ROOT/'source/SO101_base_assembly_frame.step')).val()
    modified=bores(src,ARM_HOLES,p['hole_diameter']/2,-1,20)
    for side,name in [(-1,'left'),(1,'right')]:
        parts[f'{name}_SO101_base_modified']=modified.translate((side*p['arm_x'],p['arm_y'],top))
    return p,parts

def main():
    p,parts=build(json.loads((ROOT/'parameters.json').read_text()) if (ROOT/'parameters.json').exists() else None)
    (ROOT/'parameters.json').write_text(json.dumps(p,indent=2)+'\n')
    assy=cq.Assembly(name='compact_body')
    manifest=[];mesh_repairs=[]
    for name,s in parts.items():
        assert s.isValid() and len(s.Solids())==1 and s.Volume()>0,name
        col=cq.Color(.15,.30,.24) if any(q in name for q in ['column','head_shell']) else cq.Color(.86,.85,.79)
        if name=='face_panel': col=cq.Color(.08,.10,.10)
        assy.add(s,name=name,color=col)
        cq.exporters.export(s,str(ROOT/'cad'/f'{name}.step'))
        # Print files use local axes and sit on z=0. Assembly STEP retains datums.
        bb=s.BoundingBox(); centered=s.translate((-(bb.xmin+bb.xmax)/2,-(bb.ymin+bb.ymax)/2,-bb.zmin))
        if not name.startswith('standoff'):
            path=ROOT/'cad'/f'{name}.stl'
            cq.exporters.export(centered,str(path),tolerance=.08,angularTolerance=.12)
            mesh=trimesh.load(path,force='mesh')
            keep=mesh.nondegenerate_faces(height=1e-9)
            if not keep.all():
                removed=int((~keep).sum());max_area=float(max(mesh.area_faces[~keep],default=0))
                if max_area>1e-7:raise ValueError('Nontrivial STL face repair forbidden')
                before=mesh.volume
                mesh.update_faces(keep);mesh.remove_unreferenced_vertices()
                if not mesh.is_watertight:raise ValueError('STL remains open after zero-area cleanup')
                mesh.export(path)
                mesh_repairs.append(dict(part=name,removed_zero_area_triangles=removed,max_removed_triangle_area_mm2=max_area,volume_change_mm3=float(mesh.volume-before),operation='remove zero-area tessellation triangles only; no hole filling or vertex welding'))
        manifest.append(dict(name=name,volume_mm3=s.Volume(),bbox_mm=[bb.xmin,bb.ymin,bb.zmin,bb.xmax,bb.ymax,bb.zmax]))
    assy.save(str(ROOT/'cad/compact_body.xbf'),'XBF')
    assy.save(str(ROOT/'cad/compact_body.step'),'STEP')
    assy.save(str(ROOT/'cad/compact_body.glb'),'GLTF',tolerance=.15)
    (ROOT/'checks/build_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    (ROOT/'checks/stl_export_cleanup.json').write_text(json.dumps(mesh_repairs,indent=2)+'\n')
    print('Saved editable model, native assembly, STEP, print STLs and manifest:',len(parts),'parts')

if __name__=='__main__': main()
