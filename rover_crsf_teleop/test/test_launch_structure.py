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

"""The launch file's structure: one container, both nodes intra-process, names, UDP settings."""

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
        '/**:\n  ros__parameters:\n    udp_bind_ip: 10.0.0.5\n    udp_port: 15000\n'
        '    udp_source_ip: 10.0.0.1\n')
    context = LaunchContext()
    context.launch_configurations.update(
        namespace='rover', log_level='INFO', use_sim=use_sim,
        rover_crsf_config_path=str(config))
    actions = teleop_launch._launch_setup(context)
    assert len(actions) == 1
    assert len(containers) == 1
    return containers[0], nodes, context


@pytest.mark.parametrize('use_sim', ['False', 'True'])
def test_receiver_and_teleop_share_one_single_threaded_container(
        teleop_launch, monkeypatch, tmp_path, use_sim):
    container, nodes, context = _setup(teleop_launch, monkeypatch, tmp_path, use_sim)

    assert container['package'] == 'rclcpp_components'
    # Not _mt: the teleop node is single-threaded by design.
    assert container['executable'] == 'component_container'
    assert container['name'] == 'rover_crsf_container'

    # Node names are unchanged from the standalone processes: RC UI clients and the
    # diagnostic aggregator key on them.
    assert set(nodes) == {'rover_crsf_udp_receiver', 'rover_crsf_teleop_node'}
    assert [node for node, _ in nodes.values()] == container['composable_node_descriptions']

    receiver = nodes['rover_crsf_udp_receiver'][1]
    teleop = nodes['rover_crsf_teleop_node'][1]
    assert receiver['plugin'] == 'rover::transport::udp::UdpReceiverNode'
    assert teleop['plugin'] == 'rover_crsf_teleop::RoverCrsfTeleopNode'
    for node in (receiver, teleop):
        assert {'use_intra_process_comms': True} in node['extra_arguments']
    # Lifecycle nodes, activated by their own `autostart` parameter: launch_ros'
    # ComposableLifecycleNode autostart never reaches a namespaced component.
    assert receiver['parameters'][0]['autostart'] is True
    assert {'autostart': True} in teleop['parameters']

    # The UDP receiver only exists on hardware.
    assert receiver['condition'].evaluate(context) is (use_sim == 'False')
    assert 'condition' not in teleop


def test_receiver_publishes_where_the_teleop_listens(teleop_launch, monkeypatch, tmp_path):
    _, nodes, _ = _setup(teleop_launch, monkeypatch, tmp_path, 'False')
    receiver = nodes['rover_crsf_udp_receiver'][1]
    assert ('udp_read', 'rc/raw_udp') in receiver['remappings']
    # UDP settings come from the teleop config (single source of truth).
    parameters = receiver['parameters'][0]
    assert parameters['ip'] == '10.0.0.5'
    assert parameters['port'] == 15000
    assert parameters['source_ip'] == '10.0.0.1'


def test_transition_event_topics_drop_the_rover_prefix(teleop_launch, monkeypatch, tmp_path):
    # Topic names carry no rover_ prefix; the nodes keep theirs.
    _, nodes, _ = _setup(teleop_launch, monkeypatch, tmp_path, 'False')
    for name in ('rover_crsf_udp_receiver', 'rover_crsf_teleop_node'):
        topic = name.removeprefix('rover_') + '/transition_event'
        assert ('~/transition_event', topic) in nodes[name][1]['remappings']


def test_receiver_only_accepts_the_router_by_default(teleop_launch, monkeypatch, tmp_path):
    """The shipped config must keep the source filter: CRSF is unauthenticated."""
    config = Path(__file__).resolve().parents[1] / 'config' / 'rover_crsf_teleop.yaml'
    bind_ip, port, source_ip = teleop_launch._udp_settings(
        LaunchContext(), _Literal(str(config)))
    assert (bind_ip, port, source_ip) == ('192.168.1.201', 10111, '192.168.1.1')


def test_a_blank_source_ip_keeps_the_router(teleop_launch, tmp_path):
    """A blank YAML value loads as None; it must not silently open the RC port to anyone."""
    config = tmp_path / 'blank.yaml'
    config.write_text('/**:\n  ros__parameters:\n    udp_source_ip:\n')
    _, _, source_ip = teleop_launch._udp_settings(LaunchContext(), _Literal(str(config)))
    assert source_ip == '192.168.1.1'


def test_an_explicit_empty_source_ip_accepts_any(teleop_launch, tmp_path):
    config = tmp_path / 'any.yaml'
    config.write_text('/**:\n  ros__parameters:\n    udp_source_ip: ""\n')
    _, _, source_ip = teleop_launch._udp_settings(LaunchContext(), _Literal(str(config)))
    assert source_ip == ''


class _Literal:
    """Stands in for a LaunchConfiguration that resolves to a fixed path."""

    def __init__(self, value):
        self._value = value

    def perform(self, context):
        return self._value
