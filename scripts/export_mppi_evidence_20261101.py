from pathlib import Path
import shutil,json,csv,hashlib,statistics,subprocess,platform,os
src=Path('/home/misys/shared_dir/svg_mppi_ros2_20261031');repo=Path('/home/misys/ADP-project-experiments');out=repo/'analyses/EXP-008';runs=repo/'runs/EXP-008';cfg=repo/'configs/EXP-008'
for p in [out,runs,cfg]:p.mkdir(parents=True,exist_ok=True)
allowed={'.json','.yaml','.yml','.csv','.md','.txt','.log','.png','.jsonl','.py','.cpp','.hpp','.inc','.sh','.xml'}
raw=[];rows=[]
for p in sorted((src/'manual_runs').iterdir()):
 if not p.is_dir():continue
 dest=runs/p.name;dest.mkdir(exist_ok=True)
 for f in p.iterdir():
  if f.is_file() and f.suffix in allowed:shutil.copy2(f,dest/f.name)
 meta=p/'bag/metadata.yaml'
 if meta.exists():shutil.copy2(meta,dest/'bag_metadata.yaml')
 raw.append({'session':p.name,'local_path':str(p),'bag_files':[{'name':f.name,'bytes':f.stat().st_size} for f in (p/'bag').glob('*') if f.is_file()],'raw_bags_uploaded':False})
 session=json.loads((p/'session.json').read_text()) if (p/'session.json').exists() else {}
 s=json.loads((p/'summary.json').read_text()) if (p/'summary.json').exists() else {}
 results=list(p.glob('race_result*.json'));race=json.loads(results[0].read_text()) if results else {}
 full=[v['time'] for v in race.get('laps',[]) if not v.get('partial',False)]
 rows.append({'session':p.name,'solver_version':session.get('solver_version','upstream'),'mode':session.get('mode'),'speed_m_s':session.get('reference_speed'),'obstacles':session.get('obstacles'),'seed':session.get('seed'),'target_laps':session.get('laps'),'laps_completed':race.get('laps_completed',s.get('laps_completed','')),'mean_completed_lap_s':statistics.mean(full) if full else '', 'best_lap_s':min(full) if full else '', 'runner_reason':s.get('reason','not_started_or_unclassified'),'runner_collisions':len(s.get('collisions',[])) if s else '', 'whole_record_collisions':race.get('collisions',{}).get('total',''),'cycle_p95_ms':s.get('cycle_ms',{}).get('p95',''),'cycle_over25_fraction':s.get('cycle_ms',{}).get('over_25ms_fraction',''),'record':f'../../runs/EXP-008/{p.name}'})
with (out/'results_summary.csv').open('w') as f:
 w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
(out/'local_bag_inventory.json').write_text(json.dumps(raw,indent=2))
for d in ['tuning','visual']:
 shutil.copytree(src/d,cfg/d,dirs_exist_ok=True,ignore=shutil.ignore_patterns('__pycache__','host-display.txt'))
shutil.copytree(src/'tools',cfg/'reproduction/tools',dirs_exist_ok=True,ignore=shutil.ignore_patterns('__pycache__'))
shutil.copy2(src/'manual.sh',cfg/'reproduction/manual.sh')
for workspace in ['ws','ws_repro','ws_speed','ws_clearance']:
 shutil.copytree(src/workspace/'src',cfg/'reproduction'/workspace/'src',dirs_exist_ok=True,ignore=shutil.ignore_patterns('__pycache__'))
shutil.copy2(Path('/home/misys/shared_dir/svg_mppi_20261031/source/LICENSE'),cfg/'reproduction/LICENSE.upstream')
for sub in ['sampler_repro_20261031','speed3_failure_review_20261101']:
 target=out/'diagnostics'/sub;target.mkdir(parents=True,exist_ok=True)
 for f in (src/'diagnostics'/sub).iterdir():
  if f.is_file() and f.suffix in allowed:shutil.copy2(f,target/f.name)
checks=out/'diagnostics/clearance_checks';checks.mkdir(parents=True,exist_ok=True)
for name in ['input_check.json','cost_probe.cpp','cost_probe_result.log','startup_check.log','build.log']:
 f=src/'ws_clearance'/name
 if f.exists():shutil.copy2(f,checks/name)
# Snapshot only dependencies touched by this analysis; this is not a race_stack commit.
race=Path('/home/misys/forza_ws/race_stack');dep=cfg/'race_support';dep.mkdir(exist_ok=True)
for rel in ['stack_master/config/SIM/sim_params.yaml','stack_master/config/SIM/sim.yaml','stack_master/maps/hyhall12/start_pose.yaml','stack_master/maps/hyhall12/global_waypoints.json','stack_master/maps/hyhall12/hyhall12_orig.yaml','base_system/f110_simulator/f1tenth_gym_ros/f1tenth_gym_ros/gym_bridge.py','utilities/nodes/race_manager/race_manager/obstacle_spawner.py','utilities/nodes/race_manager/config/race_manager_params.yaml']:
 f=race/rel;target=dep/rel;target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(f,target)
env={'platform':platform.platform(),'os_release':Path('/etc/os-release').read_text(),'cpu_affinity_count':len(os.sched_getaffinity(0)),'cpu_max':Path('/sys/fs/cgroup/cpu.max').read_text(),'meminfo':Path('/proc/meminfo').read_text().splitlines()[:3],'container_marker':Path('/.dockerenv').exists(),'source_root':str(src),'upstream_commit':'f4fb7b74f12d4568790213aab26dd6f0538d1925','ros':'Humble / native ROS2 adapter; official upstream is ROS1 Noetic','source_snapshots_are_current':'Per-run conditions/config/binary hashes retained; current source variants are separate.'}
(out/'environment.json').write_text(json.dumps(env,indent=2))
print('runs',len(rows),'files',sum(1 for f in runs.rglob('*') if f.is_file()))
