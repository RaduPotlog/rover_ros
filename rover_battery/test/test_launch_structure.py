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

"""The launch file's structure: BMS receiver and battery node share a container, intra-process."""

import importlib.util
from pathlib import Path

import pytest
import yaml


@pytest.fixture
def battery_launch(monkeypatch):
    path = Path(__file__).resolve().parents[1] / 'launch' / 'rover_battery.launch.py'
    spec = importlib.util.spec_from_file_location('rover_battery_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    captured = {'containers': [], 'nodes': {}}
    original_container = module.ComposableNodeContainer

    def capture_container(**kwargs):
        captured['containers'].append(kwargs)
        return original_container(**kwargs)

    def capture(original):
        def wrapper(**kwargs):
            node = original(**kwargs)
            captured['nodes'][kwargs['name']] = (original.__name__, node, kwargs)
            return node
        return wrapper

    monkeypatch.setattr(module, 'ComposableNodeContainer', capture_container)
    monkeypatch.setattr(module, 'ComposableNode', capture(module.ComposableNode))
    module.generate_launch_description()
    return captured


def test_receiver_and_battery_node_share_one_container(battery_launch):
    assert len(battery_launch['containers']) == 1
    container = battery_launch['containers'][0]
    assert container['name'] == 'rover_battery_container'
    assert container['executable'] == 'component_container'

    nodes = battery_launch['nodes']
    # Names unchanged: config/rover_battery.yaml and the diagnostic aggregator key on them.
    assert set(nodes) == {'rover_udp_battery_receiver_node', 'rover_battery_node'}
    assert ({id(node) for _, node, _ in nodes.values()}
            == {id(node) for node in container['composable_node_descriptions']})
    for _, _, kwargs in nodes.values():
        assert {'use_intra_process_comms': True} in kwargs['extra_arguments']

    _, _, receiver = nodes['rover_udp_battery_receiver_node']
    assert receiver['plugin'] == 'rover::transport::udp::UdpReceiverNode'
    # A lifecycle node activated by its own `autostart` parameter: launch_ros'
    # ComposableLifecycleNode autostart never reaches a namespaced component.
    assert {'autostart': True} in receiver['parameters']
    assert receiver['remappings'] == [('udp_read', 'rover_battery_udp_data')]

    _, _, battery = nodes['rover_battery_node']
    assert battery['plugin'] == 'rover_battery::RoverBatteryNode'


def test_config_has_a_section_for_each_node(battery_launch):
    """Both nodes read config/rover_battery.yaml; each needs its own section there."""
    config_path = Path(__file__).resolve().parents[1] / 'config' / 'rover_battery.yaml'
    config = yaml.safe_load(config_path.read_text())
    for name in battery_launch['nodes']:
        assert f'/**/{name}' in config
