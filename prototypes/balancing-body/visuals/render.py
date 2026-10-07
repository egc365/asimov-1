"""Render real CAD and source meshes through MuJoCo EGL; no invented image geometry."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import json,pathlib,numpy as np,mujoco
from PIL import Image,ImageDraw,ImageFont
ROOT=pathlib.Path(__file__).resolve().parents[1]
OUT=ROOT/'visuals'
def render(filename,azimuth,elevation=-10,parked=False,width=1200,height=1400,focus=None,exploded=False):
 m=mujoco.MjModel.from_xml_path(str(ROOT/'simulation'/('parked.xml' if parked else 'balanced.xml')));d=mujoco.MjData(m)
 if focus=='base':
  for i in range(m.ngeom):
   name=m.geom(i).name or ''
   if name.endswith('_visual') or name.endswith('_allowance_geom'):m.geom_group[i]=4
 if exploded:
  m.body_pos[m.body('pelvis_link').id,2]+=.13;m.body_pos[m.body('PART-103_interface_blank').id,2]+=.065
 mujoco.mj_forward(m,d)
 m.vis.global_.offwidth=width;m.vis.global_.offheight=height
 renderer=mujoco.Renderer(m,height,width);cam=mujoco.MjvCamera();cam.type=mujoco.mjtCamera.mjCAMERA_FREE;cam.lookat[:]=[0,0,.52];cam.distance=1.7;cam.azimuth=azimuth;cam.elevation=elevation
 if focus=='base':cam.lookat[:]=[0,0,.16];cam.distance=.8
 if exploded:cam.lookat[:]=[0,0,.59];cam.distance=1.95
 option=mujoco.MjvOption();option.geomgroup[:]=0;option.geomgroup[0]=1;option.geomgroup[2]=1
 renderer.update_scene(d,cam,scene_option=option);renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW]=1
 img=Image.fromarray(renderer.render());img.save(OUT/filename);renderer.close();return img
if __name__=='__main__':
 render('assembly_front.png',35,-10);render('assembly_side.png',90,-5);render('assembly_parked.png',35,-15,True)
 render('assembly_rear.png',215,-12);render('base_detail.png',35,-20,focus='base');render('base_top.png',45,-65,focus='base');render('assembly_exploded.png',35,-12,exploded=True)
 print('Seven actual-CAD/source-mesh renders written; exploded view offsets for illustration only')
