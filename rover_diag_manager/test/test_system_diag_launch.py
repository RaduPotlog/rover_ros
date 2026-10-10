# Copyright 2026 Mechatronics Academy
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

"""The aggregator loads the GPS / Lidar groups only while that sensor is enabled."""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import OpaqueFunction

import pytest
import yaml

PACKAGE = Path(__file__).resolve().parents[1]


@pytest.fixture
def diag_launch(monkeypatch):
    path = PACKAGE / 'launch' / 'system_diag.launch.py'
    spec = importlib.util.spec_from_file_location('system_diag_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    nodes = {}
    original_node = module.Node

    def capture_node(**kwargs):
        nodes[kwargs['name']] = kwargs
        return original_node(**kwargs)

    monkeypatch.setattr(module, 'Node', capture_node)
    return module, nodes


def _aggregator_files(module, nodes, use_gps, use_lidar):
    description = module.generate_launch_description()
    setup = next(e for e in description.entities if isinstance(e, OpaqueFunction))
    context = LaunchContext()
    context.launch_configurations.update(use_gps=use_gps, use_lidar=use_lidar)
    setup.execute(context)
    parameters = nodes['rover_diagnostic_aggregator']['parameters']
    # The first entry is the base file (a launch argument); the rest are the optional groups.
    return [p.substitutions[-1][0].text for p in parameters[1:]]


@pytest.mark.parametrize('value, expected', [
    ('true', True), ('True', True), ('1', True), ('yes', True), (' ON ', True),
    ('false', False), ('0', False), ('', False), ('nope', False)])
def test_flag_parsing(diag_launch, value, expected):
    module, _ = diag_launch
    assert module._flag(value) is expected


@pytest.mark.parametrize('use_gps, use_lidar, expected', [
    ('false', 'false', []),
    ('true', 'false', ['diagnostic_aggregator_gps.yaml']),
    ('false', 'true', ['diagnostic_aggregator_lidar.yaml']),
    ('true', 'true', ['diagnostic_aggregator_gps.yaml', 'diagnostic_aggregator_lidar.yaml']),
])
def test_aggregator_loads_only_the_enabled_sensor_groups(diag_launch, use_gps, use_lidar, expected):
    module, nodes = diag_launch
    assert _aggregator_files(module, nodes, use_gps, use_lidar) == expected


def _groups(name):
    params = yaml.safe_load((PACKAGE / 'config' / name).read_text())['/**']['ros__parameters']
    return {key for key, value in params.items() if isinstance(value, dict)}


def test_sensor_groups_live_only_in_their_own_files():
    assert not {'gps', 'lidar'} & _groups('diagnostic_aggregator.yaml')
    assert _groups('diagnostic_aggregator_gps.yaml') == {'gps'}
    assert _groups('diagnostic_aggregator_lidar.yaml') == {'lidar'}
