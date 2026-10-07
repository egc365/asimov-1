"""Three publication-quality evidence figures; plotted values come from saved checks."""
import pathlib,json,sys,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
ROOT=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'simulation'))
from balance import simulate
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':11,'axes.spines.top':False,'axes.spines.right':False,'axes.labelcolor':'#23333c','text.color':'#23333c','axes.titleweight':'bold','figure.facecolor':'#f7f8f5','axes.facecolor':'#f7f8f5','savefig.facecolor':'#f7f8f5','axes.edgecolor':'#78858b','grid.color':'#d8dedb'})
nom=np.loadtxt(ROOT/'checks/contact_trace.csv',delimiter=',',skiprows=1);plan=np.loadtxt(ROOT/'checks/nominal_trace.csv',delimiter=',',skiprows=1)
_,delayed=simulate(delay=.05,save=True);_,off=simulate(motor_off_at=1,push_N=10,save=True)
fig,ax=plt.subplots(2,2,figsize=(13,8),layout='constrained')
total=json.loads((ROOT/'checks/mass_properties.json').read_text())['total_mass_kg'];fig.suptitle(f'BALANCE VALIDATION  /  {total:.2f} kg integrated model',fontsize=20,x=.035,ha='left')
ax[0,0].plot(nom[:,0],np.rad2deg(nom[:,3]),label='3D contact model',color='#226b61',lw=2.5);ax[0,0].plot(plan[:,0],np.rad2deg(plan[:,3]),label='Planar model',color='#7ba6a1',ls='--');ax[0,0].set(title='5° disturbance recovers',xlabel='Time (s)',ylabel='Body pitch (°)',xlim=(0,4));ax[0,0].legend(frameon=False)
ax[0,1].plot(nom[:,0],nom[:,-1],color='#226b61',lw=2);ax[0,1].axhline(6,color='#b87954',ls='--');ax[0,1].axhline(-6,color='#b87954',ls='--');ax[0,1].set(title='Recovery reaches the assumed torque limit',xlabel='Time (s)',ylabel='Torque per wheel (Nm)',xlim=(0,2),ylim=(-7,7))
ax[1,0].plot(delayed[:,0],np.rad2deg(delayed[:,3]),color='#a55542',lw=2);ax[1,0].set(title='50 ms delay: oscillatory, fails recovery criterion',xlabel='Time (s)',ylabel='Pitch (°)')
ax[1,1].plot(off[:,0],np.rad2deg(off[:,3]),color='#a55542',lw=2);ax[1,1].axvline(1,color='#78858b',ls='--');ax[1,1].set(title='Power cut at 1 s: falls past 45°',xlabel='Time (s)',ylabel='Pitch (°)')
for a in ax.ravel():a.grid(axis='y',alpha=.7)
fig.savefig(ROOT/'visuals/balance_evidence.png',dpi=180);plt.close(fig)
rows=json.loads((ROOT/'checks/planar_sweep.json').read_text());angles=[2,5,10,15,20,25,30];torques=[1,2,3,6];mat=np.zeros((len(torques),len(angles)))
for r in rows[:28]:mat[torques.index(r['torque_limit_Nm_each']),angles.index(r['initial_deg'])]=r['recovered']
fig,ax=plt.subplots(figsize=(12,6.5),layout='constrained');ax.imshow(mat,cmap=ListedColormap(['#d6c4b6','#226b61']),vmin=0,vmax=1,aspect='auto');ax.set_xticks(range(7),[str(x)+'°' for x in angles]);ax.set_yticks(range(4),[str(x)+' Nm' for x in torques]);ax.set(xlabel='Initial lean angle, stationary start',ylabel='Assumed torque limit per wheel',title='PLANAR RECOVERY SCREEN  /  28 angle–torque cases')
for i in range(4):
 for j in range(7):ax.text(j,i,'Recovered' if mat[i,j] else 'Failed',ha='center',va='center',color='white' if mat[i,j] else '#4f4034',fontsize=11)
fig.text(.05,.015,'8 s criterion: |pitch| < 1° and |pitch rate| < 0.02 rad/s. Level floor, μ=0.6, 5 ms loop and delay. Not a certified operating envelope.',fontsize=9)
fig.savefig(ROOT/'visuals/recovery_envelope.png',dpi=180);plt.close(fig)
mass=json.loads((ROOT/'checks/mass_properties.json').read_text());manifest=json.loads((ROOT/'cad/concept/manifest.json').read_text());alu=sum(r['mass_kg'] for r in manifest if r['material']=='aluminum' and 'parking_' not in r['id']);parking=sum(r['mass_kg'] for r in manifest if 'parking_' in r['id'])
labels=['Asimov upper body','Two wheel/motor allowances','New aluminum frame','Battery allowance','Two hand allowances','Parking supports from CAD','Electronics allowance','Cables / fasteners allowance'];values=[19.332296427812093,3,alu,2.3,1,parking,.6,.35]
fig,ax=plt.subplots(figsize=(12,6.5),layout='constrained');bars=ax.barh(labels[::-1],values[::-1],color=['#8baba4']*7+['#226b61']);ax.bar_label(bars,labels=[f'{v:.2f} kg' for v in values[::-1]],padding=7);ax.set(xlabel='Mass (kg)',xlim=(0,22),title='MASS BUDGET  /  source values and explicit allowances');ax.grid(axis='x',alpha=.5);ax.set_axisbelow(True);fig.text(.04,.015,f"Total: {mass['total_mass_kg']:.2f} kg. Source inertials preserved; frame mass from CAD volume. Other entries require weighing selected hardware.",fontsize=10)
fig.savefig(ROOT/'visuals/mass_budget.png',dpi=180);plt.close(fig)
print('Three evidence figures written; total visual set: ten images')
