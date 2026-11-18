"""Map selection shared by the manual harness; old sessions default to HYHALL12."""
import hashlib
import re
from pathlib import Path
import yaml

RACE = Path('/home/misys/forza_ws/race_stack')

def resolve_map(name='hyhall12'):
    if not re.fullmatch(r'[A-Za-z0-9_-]+', name):
        raise ValueError('Invalid map name')
    folder = RACE/'stack_master/maps'/name
    map_yaml = folder/(name+'_orig.yaml')
    if not map_yaml.is_file():
        map_yaml = folder/(name+'.yaml')
    meta = yaml.safe_load(map_yaml.read_text())
    files = {'map_yaml': map_yaml, 'map_image': (folder/meta['image']).resolve(),
             'waypoints_file': folder/'global_waypoints.json',
             'start_pose_file': folder/'start_pose.yaml',
             'speed_scaling_file': folder/'speed_scaling.yaml'}
    for path in files.values():
        if not path.is_file():
            raise FileNotFoundError(path)
    return {'map_name': name, **{k:str(v.resolve()) for k,v in files.items()}}

def asset_hashes(context):
    return {context[k]:hashlib.sha256(Path(context[k]).read_bytes()).hexdigest()
            for k in ['map_yaml','map_image','waypoints_file','start_pose_file','speed_scaling_file']}

def session_map(session):
    if 'map_yaml' not in session:
        return resolve_map('hyhall12')
    context = {k:session[k] for k in ['map_name','map_yaml','map_image','waypoints_file',
                                    'start_pose_file','speed_scaling_file']}
    for path,digest in session.get('map_asset_sha256',{}).items():
        if hashlib.sha256(Path(path).read_bytes()).hexdigest() != digest:
            raise RuntimeError('Map asset changed after preparation: '+path+'; prepare a new session')
    return context

def simulator_config(context):
    sim = yaml.safe_load((RACE/'stack_master/config/SIM/sim.yaml').read_text())
    params = sim['bridge']['ros__parameters']
    params.update(yaml.safe_load(Path(context['start_pose_file']).read_text()))
    params.update(map_path=context['map_yaml'],
                  sim_params=str(RACE/'stack_master/config/SIM/sim_params.yaml'),
                  ego_drive_topic='/mppi_sim/drive', kb_teleop=False)
    return sim
