"""Independent polygon-edge oracle, not the SAT implementation under test."""
import json,math,subprocess
from pathlib import Path
import numpy as np
root=Path(__file__).resolve().parent
rng=np.random.default_rng(20261118)
cases=[[0,0,0,.30,0],[0,0,math.pi/2,.30,0],[0,0,math.pi/2,0,.30],[0,0,0,.375,0],[0,0,0,1,1],[4.9,0,0,0,0]]
cases+=np.column_stack([rng.uniform(-1,1,(12000,2)),rng.uniform(-math.pi,math.pi,12000),rng.uniform(-1,1,(12000,2))]).tolist()
def cross(a,b):return a[0]*b[1]-a[1]*b[0]
def inside(p,poly):return all(cross(poly[(i+1)%4]-poly[i],p-poly[i])>=-1e-13 for i in range(4))
def pointsegment(p,a,b):
 d=b-a;t=np.clip(np.dot(p-a,d)/np.dot(d,d),0,1);return np.linalg.norm(p-(a+t*d))
def oracle(case):
 x,y,yaw,ox,oy=case;c,s=math.cos(yaw),math.sin(yaw)
 body=np.array([[-.35,-.15],[.35,-.15],[.35,.15],[-.35,.15]])@np.array([[c,s],[-s,c]])+np.array([x,y])
 cell=np.array([[-.025,-.025],[.025,-.025],[.025,.025],[-.025,.025]])+np.array([ox,oy])
 if (abs(body)>=5).any():return 100.,5.
 hit=any(inside(p,cell) for p in body) or any(inside(p,body) for p in cell)
 for i in range(4):
  a,b=body[i],body[(i+1)%4]
  for j in range(4):
   p,q=cell[j],cell[(j+1)%4]
   if cross(b-a,p-a)*cross(b-a,q-a)<0 and cross(q-p,a-p)*cross(q-p,b-p)<0:hit=True
 if hit:return 100.,5.
 distance=min(pointsegment(p,poly[i],poly[(i+1)%4]) for points,poly in [(body,cell),(cell,body)] for p in points for i in range(4))
 return 0.,5*max(0,1-distance/.30)**2
proc=subprocess.run([str(root/'test_geometry')],input=''.join(' '.join(map(str,c))+'\n' for c in cases),text=True,capture_output=True,check=True)
actual=np.array([list(map(float,l.split())) for l in proc.stdout.splitlines()]);expected=np.array([oracle(c) for c in cases])
errors=np.abs(actual-expected)
result={'cases':len(cases),'collision_mismatches':int((errors[:,0]>1e-8).sum()),'maximum_clearance_cost_error':float(errors[:,1].max()),'examples':actual[:6].tolist()}
(root/'geometry_test.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
assert result['collision_mismatches']==0 and result['maximum_clearance_cost_error']<1e-8
