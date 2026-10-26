"""Load an explicit, immutable obstacle layout for controlled experiments."""
import json
import math
from types import SimpleNamespace


def load_fixed_layout(path, *, map_name, size, count):
    with open(path) as stream:
        data = json.load(stream)
    if data.get('map') != map_name:
        raise ValueError('fixed layout map does not match waypoint map')
    layout_size = float(data['obstacle_size_m'])
    if not math.isfinite(layout_size) or not math.isclose(layout_size, size, abs_tol=1e-9):
        raise ValueError('fixed layout obstacle size does not match spawner size')
    poses = data['obstacles']
    if not poses or len(poses) != count:
        raise ValueError('fixed layout count must match num_obstacles and be nonzero')
    result = []
    for i, pose in enumerate(poses):
        x, y, theta = (float(pose[k]) for k in ('x', 'y', 'yaw'))
        if not all(math.isfinite(v) for v in (x, y, theta)):
            raise ValueError('fixed layout poses must be finite')
        result.append(SimpleNamespace(x=x, y=y, theta=theta, id=f'fixed-{i+1}'))
    return tuple(result)
