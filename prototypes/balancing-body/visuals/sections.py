"""Exact STEP plane sections and measured drive trace; no invented geometry."""
import pathlib,json,numpy as np,cadquery as cq,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from OCP.BRepAlgoAPI import BRepAlgoAPI_Section
from OCP.gp import gp_Pln,gp_Pnt,gp_Dir
ROOT=pathlib.Path(__file__).resolve().parents[1]
M=json.loads((ROOT/'cad/concept/manifest.json').read_text())
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':11,'axes.spines.top':False,'axes.spines.right':False})
fig,axs=plt.subplots(1,3,figsize=(15,7),constrained_layout=True)
for ax,y,include,title in [(axs[0],167,['PART-106_left_torque_clamp_lower','PART-106_left_torque_clamp_cap'],'Split clamp · Y = 167 mm'),(axs[1],72.5,['PART-105_left_pelvis_cradle','PART-103_interface_blank'],'Source-pattern cradle · Y = 72.5 mm'),(axs[2],0,['PART-101_deck','PART-102_column','PART-103_interface_blank','PART-202_battery_ENVELOPE'],'Central frame · Y = 0 mm')]:
 for pid in include:
  s=cq.importers.importStep(str(ROOT/'cad/concept'/f'{pid}.step')).val()
  op=BRepAlgoAPI_Section(s.wrapped,gp_Pln(gp_Pnt(0,y,0),gp_Dir(0,1,0)),False);op.Build()
  for edge in cq.Shape.cast(op.Shape()).Edges():
   points=edge.sample(40)[0];p=np.array([[v.x,v.z] for v in points]);ax.plot(p[:,0],p[:,1],lw=1.8,color='#158b82' if 'ENVELOPE' not in pid else '#ac8536')
 ax.set_aspect('equal');ax.set_title(title,fontweight='bold');ax.set_xlabel('X forward (mm)');ax.set_ylabel('Z above axle (mm)');ax.grid(alpha=.14)
axs[0].annotate('Ø14.15 mm bore\n14 mm shaft requirement',xy=(0,7.075),xytext=(-38,48),arrowprops={'arrowstyle':'->','color':'#253b50'},fontsize=10)
axs[0].annotate('1.5 mm split',xy=(28,0),xytext=(-25,-48),arrowprops={'arrowstyle':'->','color':'#253b50'},fontsize=10)
axs[0].set_ylim(-65,70)
axs[1].set_ylim(200,370)
axs[1].text(.03,.02,'8 holes · Ø4.3 mm\n100.2 mm PCD\nSource case datum unverified',transform=axs[1].transAxes,fontsize=10)
axs[2].annotate('8 mm deck',xy=(75,110),xytext=(65,156),arrowprops={'arrowstyle':'->'},fontsize=10)
axs[2].text(.03,.94,'60 × 80 mm post\n3 mm wall\nBattery envelope below',va='top',transform=axs[2].transAxes,fontsize=10)
fig.suptitle('Actual solid sections · provisional mounting interfaces',fontsize=20,fontweight='bold');fig.savefig(ROOT/'visuals/interface_sections.png',dpi=170,facecolor='white');plt.close(fig)
d=np.genfromtxt(ROOT/'checks/driving_trace.csv',delimiter=',',names=True)
fig,ax=plt.subplots(3,1,figsize=(11,9),sharex=True,constrained_layout=True)
ax[0].plot(d['time_s'],d['path_distance_m']*1000,label='Measured model path',color='#157d74');ax[0].plot(d['time_s'],d['reference_distance_m']*1000,'--',label='Reference',color='#46576c');ax[0].set_ylabel('Path (mm)');ax[0].legend(loc='upper left');ax[0].text(.99,.15,'Final error +34.37 mm\n30 mm criterion: FAIL',ha='right',transform=ax[0].transAxes,color='#9c3b27',fontweight='bold')
ax[1].plot(d['time_s'],d['yaw_deg'],color='#157d74');ax[1].set_ylabel('Yaw (degrees)');ax[1].axhline(34.37746770785202,color='#46576c',ls='--',alpha=.7);ax[1].text(.98,.2,'Final heading error 0.118°',ha='right',transform=ax[1].transAxes)
ax[2].plot(d['time_s'],d['COM_pitch_deg'],label='COM pitch',color='#157d74');ax[2].plot(d['time_s'],d['left_torque_Nm'],label='Left wheel Nm',alpha=.65,color='#d58a3d');ax[2].plot(d['time_s'],d['right_torque_Nm'],label='Right wheel Nm',alpha=.65,color='#476da0');ax[2].set_ylabel('Degrees / Nm');ax[2].set_xlabel('Time (s)');ax[2].legend(loc='upper right')
for a in ax:a.grid(alpha=.15)
fig.suptitle('Integrated drive + yaw + arm reach · 3D contact simulation',fontsize=18,fontweight='bold');fig.savefig(ROOT/'visuals/driving_evidence.png',dpi=160,facecolor='white');plt.close(fig)
print('Exact section views and integrated driving failure chart written')
