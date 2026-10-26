import json
import math
from types import SimpleNamespace
import pytest
from race_manager.fixed_layout import load_fixed_layout
from race_manager.obstacle_spawner import ObstacleManager


def layout(tmp_path, **updates):
    data = dict(map='hyhall12', obstacle_size_m=0.3,
                obstacles=[dict(x=1., y=2., yaw=0.7), dict(x=3., y=4., yaw=-1.)])
    data.update(updates)
    path = tmp_path / 'layout.json'
    path.write_text(json.dumps(data))
    return path


def test_exact_poses_and_unchanged_after_lap_callback(tmp_path):
    fixed = load_fixed_layout(layout(tmp_path), map_name='hyhall12', size=.3, count=2)
    published = []
    fake = SimpleNamespace(fixed_layout=fixed, lap=0, current=None,
                           republish=lambda **kw: published.append(kw),
                           get_parameter=lambda key: SimpleNamespace(value='layout.json'),
                           get_logger=lambda: SimpleNamespace(info=lambda msg: None))
    fake.place = lambda: ObstacleManager.place(fake)
    fake.place()
    original = fake.current
    ObstacleManager.lap_callback(fake, SimpleNamespace(data=1))
    assert fake.current is original
    assert [(p.x, p.y, p.theta) for p in original] == [(1., 2., .7), (3., 4., -1.)]
    assert len(published) == 2


@pytest.mark.parametrize('updates,kwargs', [
    ({'map': 'other'}, {}),
    ({'obstacle_size_m': .4}, {}),
    ({'obstacles': []}, {}),
    ({}, {'count': 4}),
    ({'obstacles': [dict(x=math.nan, y=0, yaw=0), dict(x=0, y=0, yaw=0)]}, {}),
])
def test_reject_inconsistent_layout(tmp_path, updates, kwargs):
    options = dict(map_name='hyhall12', size=.3, count=2)
    options.update(kwargs)
    with pytest.raises(ValueError):
        load_fixed_layout(layout(tmp_path, **updates), **options)
