"""Known simulation starts initialize PF locally without consuming live GT."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import math
import pytest
from particle_filter.particle_filter import ParticleFiler
from particle_filter import utils


def launch_module():
    path = Path(__file__).parents[1] / 'launch/localize_launch.py'
    spec = importlib.util.spec_from_file_location('localize_prior_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_real_or_unspecified_launch_does_not_use_sim_start(tmp_path):
    (tmp_path / 'start_pose.yaml').write_text('sx: 1\nsy: 2\nstheta: 0.3\n')
    assert launch_module().start_pose_parameters(str(tmp_path/'map.yaml'), False) == {}


def test_sim_prior_uses_same_map_pose_as_bridge(tmp_path):
    (tmp_path / 'start_pose.yaml').write_text('sx: 10.8\nsy: -0.5\nstheta: 0.36\n')
    params = launch_module().start_pose_parameters(str(tmp_path/'map_orig.yaml'), True)
    assert params == dict(initial_pose_enabled=True, initial_pose_x=10.8,
                          initial_pose_y=-0.5, initial_pose_yaw=0.36)


def test_maps_without_start_file_keep_global_initialization(tmp_path):
    assert launch_module().start_pose_parameters(str(tmp_path/'map.yaml'), True) == {}


def test_nonfinite_start_is_rejected(tmp_path):
    (tmp_path / 'start_pose.yaml').write_text('sx: .nan\nsy: 2\nstheta: 0.3\n')
    with pytest.raises(ValueError):
        launch_module().start_pose_parameters(str(tmp_path/'map.yaml'), True)


def test_startup_selects_local_prior_once():
    params = dict(initial_pose_enabled=True, initial_pose_x=10.8,
                  initial_pose_y=-0.5, initial_pose_yaw=0.36)
    calls = []
    fake = SimpleNamespace(
        get_parameter=lambda key: SimpleNamespace(value=params[key]),
        get_logger=lambda: SimpleNamespace(info=lambda message: None),
        initialize_particles_pose=lambda pose: calls.append(pose),
        initialize_global=lambda: pytest.fail('Global init bypassed supplied prior'))
    ParticleFiler.initialize_startup(fake)
    assert len(calls) == 1
    assert calls[0].position.x == 10.8
    assert calls[0].position.y == -0.5
    assert math.isclose(utils.quaternion_to_angle(calls[0].orientation), 0.36)


def test_startup_keeps_global_fallback():
    calls = []
    fake = SimpleNamespace(
        get_parameter=lambda key: SimpleNamespace(value=False),
        initialize_global=lambda: calls.append('global'))
    ParticleFiler.initialize_startup(fake)
    assert calls == ['global']
