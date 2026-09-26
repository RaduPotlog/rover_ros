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

"""The launch file's structure: one container, both nodes intra-process, names unchanged."""

import importlib.util
from pathlib import Path

from launch import LaunchContext

import pytest


@pytest.fixture
def teleop_launch():
    path = Path(__file__).resolve().parents[1] / 'launch' / 'rover_crsf_teleop.launch.py'
    spec = importlib.util.spec_from_file_location('rover_crsf_teleop_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _setup(teleop_launch, monkeypatch, tmp_path, use_sim):
    """Run _launch_setup; return the container kwargs, the node kwargs by name, and the context."""
    containers = []
    nodes = {}
    original_container = teleop_launch.ComposableNodeContainer
    original_node = teleop_launch.ComposableNode

    def capture_container(**kwargs):
        containers.append(kwargs)
        return original_container(**kwargs)

    def capture_node(**kwargs):
        node = original_node(**kwargs)
        nodes[kwargs['name']] = (node, kwargs)
        return node

    monkeypatch.setattr(teleop_launch, 'ComposableNodeContainer', capture_container)
    monkeypatch.setattr(teleop_launch, 'ComposableNode', capture_node)
    config = tmp_path / 'rover_crsf_teleop.yaml'
    config.write_text(
        '/**:\n  ros__parameters:\n    serial_device: /dev/ttyTEST\n    serial_baudrate: 115200\n')
    context = LaunchContext()
    context.launch_configurations.update(
        namespace='rover', log_level='INFO', use_sim=use_sim,
        rover_crsf_config_path=str(config))
    actions = teleop_launch._launch_setup(context)
    assert len(actions) == 1
    assert len(containers) == 1
    return containers[0], nodes, context


@pytest.mark.parametrize('use_sim', ['False', 'True'])
def test_bridge_and_teleop_share_one_single_threaded_container(
        teleop_launch, monkeypatch, tmp_path, use_sim):
    container, nodes, context = _setup(teleop_launch, monkeypatch, tmp_path, use_sim)

    assert container['package'] == 'rclcpp_components'
    # Not _mt: the teleop node is single-threaded by design.
    assert container['executable'] == 'component_container'
    assert container['name'] == 'rover_crsf_container'

    # Node names are unchanged from the standalone processes: the Cockpit RC page and the
    # diagnostic aggregator key on them.
    assert set(nodes) == {'rover_crsf_serial_bridge', 'rover_crsf_teleop_node'}
    assert [node for node, _ in nodes.values()] == container['composable_node_descriptions']

    bridge = nodes['rover_crsf_serial_bridge'][1]
    teleop = nodes['rover_crsf_teleop_node'][1]
    assert bridge['plugin'] == 'rover::transport::serial::SerialBridgeNode'
    assert teleop['plugin'] == 'rover_crsf_teleop::RoverCrsfTeleopNode'
    for node in (bridge, teleop):
        assert {'use_intra_process_comms': True} in node['extra_arguments']
    # Lifecycle nodes, activated by their own `autostart` parameter: launch_ros'
    # ComposableLifecycleNode autostart never reaches a namespaced component.
    assert bridge['parameters'][0]['autostart'] is True
    assert {'autostart': True} in teleop['parameters']

    # The serial bridge only exists on hardware.
    assert bridge['condition'].evaluate(context) is (use_sim == 'False')
    assert 'condition' not in teleop


def test_bridge_publishes_where_the_teleop_listens(teleop_launch, monkeypatch, tmp_path):
    _, nodes, _ = _setup(teleop_launch, monkeypatch, tmp_path, 'False')
    bridge = nodes['rover_crsf_serial_bridge'][1]
    assert ('serial_read', 'rc/raw') in bridge['remappings']
    # Serial settings come from the teleop config (single source of truth).
    parameters = bridge['parameters'][0]
    assert parameters['device_name'] == '/dev/ttyTEST'
    assert parameters['baud_rate'] == 115200
