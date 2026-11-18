import os,subprocess,time,signal,json,sys
from pathlib import Path
from PIL import ImageGrab
root=Path(__file__).resolve().parents[1];os.chdir(root)
env=os.environ.copy();env.update(DISPLAY='localhost:77',MPPI_DISPLAY='localhost:77',XAUTHORITY=str(root/'logs/xvfb.auth'),LIBGL_ALWAYS_SOFTWARE='1',ROS_DOMAIN_ID='78')
r=subprocess.run(['bash','manual.sh','prepare','forward_mppi','--config','tuning/R01_baseline.yaml','--obstacles',os.getenv('GUI_TEST_OBSTACLES','0'),'--laps','0','--duration','8','--domain','78','--label','GUI_validation'],env=env,text=True,stdout=subprocess.PIPE,check=True)
session=Path(r.stdout.split('SESSION ')[1].splitlines()[0]);(root/'logs/manual-gui-session.txt').write_text(str(session)+'\n');children=[];handles=[]
os.environ['XAUTHORITY']=env['XAUTHORITY']
def start(phase,extra=[]):
 h=(root/'logs'/f'{session.name}-{phase}.log').open('w');handles.append(h);p=subprocess.Popen(['bash','manual.sh',phase,str(session),*extra],env=env,stdout=h,stderr=subprocess.STDOUT,start_new_session=True);children.append((phase,p));return p
try:
 start('sim',[] if os.getenv('GUI_TEST_DEFAULT_SIM')=='1' else ['--headless']);time.sleep(3)
 start('scenario');start('record')
 if os.getenv('GUI_TEST_DEFAULT_SIM')!='1':start('rviz')
 time.sleep(5)
 for name,p in children:
  if p.poll() is not None:raise RuntimeError(name+' exited '+str(p.returncode))
 controller=start('controller');deadline=time.monotonic()+45
 while time.monotonic()<deadline:
  if 'RUN ' in (root/'logs'/f'{session.name}-controller.log').read_text():break
  if controller.poll() is not None:raise RuntimeError('Controller exited before motion')
  time.sleep(.2)
 else:raise TimeoutError('Controller did not start')
 time.sleep(3)
 ImageGrab.grab(xdisplay='localhost:77').save(root/'visual/rviz-live-validation.png')
 assert controller.wait(timeout=30)==0
 summary=json.loads((session/'summary.json').read_text());assert summary['distance_m']>1 and not summary['collisions']
 (root/'logs/manual-gui-validation.json').write_text(json.dumps({'passed':True,'session':str(session),'distance_m':summary['distance_m'],'screenshot':str(root/'visual/rviz-live-validation.png'),'display':'Xvfb virtual display, software OpenGL; host :0 still requires authorization'},indent=2));print(json.dumps(summary,indent=2))
finally:
 for name,p in reversed(children):
  if p.poll() is None:
   os.killpg(p.pid,signal.SIGINT)
   try:p.wait(timeout=25)
   except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=10)
 for h in handles:h.close()
