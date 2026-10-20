# Copyright 2026 ForzaETH
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Tests for simulator occupancy-map selection."""

import importlib.util
from pathlib import Path

import f1tenth_gym_ros.map_selection as map_selection
from launch import LaunchContext


def _load_bridge_launch():
    path = Path(__file__).parents[1] / 'launch' / 'gym_bridge_launch.py'
    spec = importlib.util.spec_from_file_location('gym_bridge_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_prefers_orig_yaml_when_it_exists(tmp_path):
    """Prefer the raw SLAM map when its YAML exists."""
    base = tmp_path / 'hyhall12.yaml'
    orig = tmp_path / 'hyhall12_orig.yaml'
    base.touch()
    orig.touch()

    assert map_selection.prefer_orig_map_yaml(str(base)) == str(orig)


def test_uses_base_yaml_when_orig_does_not_exist(tmp_path):
    """Keep the configured map when there is no raw SLAM map."""
    base = tmp_path / 'glc_ot_ez.yaml'
    base.touch()

    assert map_selection.prefer_orig_map_yaml(str(base)) == str(base)


def test_map_stem_preserves_dots_in_directory_and_basename():
    """Remove only the YAML suffix from a dotted map path."""
    assert hasattr(map_selection, 'map_yaml_stem'), (
        'safe simulator map stem conversion is not implemented')
    path = '/tmp/maps.v2/hyhall.12_orig.yaml'

    assert map_selection.map_yaml_stem(path) == '/tmp/maps.v2/hyhall.12_orig'


def test_launch_shares_orig_yaml_with_bridge_and_map_server(tmp_path):
    """Resolve one map path for every simulator launch consumer."""
    base = tmp_path / 'hyhall12.yaml'
    orig = tmp_path / 'hyhall12_orig.yaml'
    base.touch()
    orig.touch()
    module = _load_bridge_launch()
    assert hasattr(module, 'configure_simulator_map'), (
        'launch-wide simulator map selection is not implemented')
    context = LaunchContext()
    context.launch_configurations['map_yaml_path'] = str(base)

    actions = module.configure_simulator_map(context)
    for action in actions:
        action.execute(context)

    assert context.launch_configurations['resolved_map_yaml_path'] == str(orig)


def test_launch_uses_fixed_pose_from_requested_map_directory(tmp_path):
    (tmp_path / 'hyhall12_orig.yaml').touch()
    (tmp_path / 'start_pose.yaml').write_text('sx: 10.8\nsy: -0.5\nstheta: 0.36\n')
    context = LaunchContext()
    context.launch_configurations['map_yaml_path'] = str(tmp_path / 'hyhall12.yaml')
    for action in _load_bridge_launch().configure_simulator_map(context):
        action.execute(context)
    assert float(context.launch_configurations['map_start_sx']) == 10.8
    assert float(context.launch_configurations['map_start_sy']) == -0.5
    assert float(context.launch_configurations['map_start_stheta']) == 0.36


def test_launch_without_fixed_pose_keeps_sim_defaults(tmp_path):
    context = LaunchContext()
    context.launch_configurations['map_yaml_path'] = str(tmp_path / 'other.yaml')
    for action in _load_bridge_launch().configure_simulator_map(context):
        action.execute(context)
    assert 'map_start_sx' not in context.launch_configurations
