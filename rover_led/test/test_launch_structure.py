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

"""The launch file's structure: the UDP senders live in the LED container, intra-process."""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions

import pytest


@pytest.fixture
def led_launch(monkeypatch):
    path = Path(__file__).resolve().parents[1] / 'launch' / 'rover_led.launch.py'
    spec = importlib.util.spec_from_file_location('rover_led_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    captured = {'containers': [], 'nodes': {}}
    original_container = module.ComposableNodeContainer
    original_node = module.ComposableNode

    def capture_container(**kwargs):
        captured['containers'].append(kwargs)
        return original_container(**kwargs)

    def capture(original):
        def wrapper(**kwargs):
            node = original(**kwargs)
            captured['nodes'][kwargs['name']] = (original, node, kwargs)
            return node
        return wrapper

    monkeypatch.setattr(module, 'ComposableNodeContainer', capture_container)
    monkeypatch.setattr(module, 'ComposableNode', capture(original_node))
    module.generate_launch_description()
    return module, captured


def _resolve(context, value):
    return perform_substitutions(context, normalize_to_list_of_substitutions(value))


def test_one_container_holds_driver_controller_and_both_senders(led_launch):
    _, captured = led_launch
    assert len(captured['containers']) == 1
    container = captured['containers'][0]
    assert container['name'] == 'rover_led_container'

    nodes = captured['nodes']
    assert set(nodes) == {
        'rover_led_driver',
        'rover_led_controller',
        'rover_udp_led_channel_1_sender_node',
        'rover_udp_led_channel_2_sender_node',
    }
    assert ({id(node) for _, node, _ in nodes.values()}
            == {id(node) for node in container['composable_node_descriptions']})
    for _, _, kwargs in nodes.values():
        assert {'use_intra_process_comms': True} in kwargs['extra_arguments']


@pytest.mark.parametrize('channel', [1, 2])
@pytest.mark.parametrize('use_sim', ['False', 'True'])
def test_udp_sender_feeds_its_channel_on_hardware_only(led_launch, channel, use_sim):
    module, captured = led_launch
    _, _, kwargs = captured['nodes'][f'rover_udp_led_channel_{channel}_sender_node']

    assert kwargs['package'] == 'rover_udp_driver'
    assert kwargs['plugin'] == 'rover::transport::udp::UdpSenderNode'
    # The sender only transmits while active; its own `autostart` parameter takes it there
    # (launch_ros' ComposableLifecycleNode autostart never reaches a namespaced component).
    assert {'autostart': True} in kwargs['parameters']
    assert kwargs['remappings'] == [('udp_write', f'udp_write/led_channel_{channel}')]

    context = LaunchContext()
    context.launch_configurations.update(robot_model='rover_a1', use_sim=use_sim)
    config = _resolve(context, kwargs['parameters'][0])
    assert config.endswith(f'/config/rover_a1_udp_led_channel_{channel}.yaml')
    assert kwargs['condition'].evaluate(context) is (use_sim == 'False')
