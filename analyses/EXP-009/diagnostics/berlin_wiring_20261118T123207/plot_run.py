import json,sys
from pathlib import Path
import numpy as np,yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from PIL import Image
p=Path(sys.argv[1]);race=Path('/home/misys/forza_ws/race_stack');a=np.loadtxt(p/'trajectory.csv',delimiter=',',skiprows=1,ndmin=2)
m=yaml.safe_load((race/'stack_master/maps/hyhall12/hyhall12_orig.yaml').read_text());im=Image.open(race/'stack_master/maps/hyhall12/hyhall12_orig.png');r=m['resolution'];x,y,_=m['origin'];wp=json.loads((race/'stack_master/maps/hyhall12/global_waypoints.json').read_text())['global_traj_wpnts_iqp']['wpnts']
fig,ax=plt.subplots(figsize=(11,8));ax.imshow(im,extent=[x,x+im.width*r,y,y+im.height*r],origin='upper',cmap='gray');ax.plot([v['x_m'] for v in wp],[v['y_m'] for v in wp],'--',label='reference',alpha=.6);ax.plot(a[:,1],a[:,2],label='GT trajectory');ax.scatter(a[0,1],a[0,2],label='start')
layouts=json.loads((p/'obstacle_layouts.json').read_text())
if layouts:
 for ob in layouts[0]['poses']:ax.scatter(ob['x'],ob['y'],color='red',marker='s')
ax.axis('equal');ax.legend();ax.set_title(p.name);fig.savefig(p/'trajectory.png',dpi=160);plt.close(fig)
fig,axs=plt.subplots(3,1,sharex=True,figsize=(11,7));axs[0].plot(a[:,0],a[:,3],label='GT vx');axs[0].plot(a[:,0],a[:,4],label='speed cmd');axs[0].legend();axs[1].plot(a[:,0],a[:,5]);axs[1].set_ylabel('steer [rad]');axs[2].plot(a[:,0],a[:,6]);axs[2].set_ylabel('predicted collision rate');axs[2].set_xlabel('wall [s]');fig.tight_layout();fig.savefig(p/'commands.png',dpi=150)
