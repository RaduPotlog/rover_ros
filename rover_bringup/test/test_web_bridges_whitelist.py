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

"""foxglove_bridge's topic_whitelist must cover what the web UIs use, and nothing else."""

import importlib.util
from pathlib import Path
import re

import pytest
import yaml


@pytest.fixture(scope='module')
def patterns():
    path = Path(__file__).resolve().parents[1] / 'launch' / 'rover_web_bridges.launch.py'
    spec = importlib.util.spec_from_file_location('rover_web_bridges_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return yaml.safe_load(module.FOXGLOVE_TOPIC_WHITELIST)


def _allowed(patterns, topic):
    # foxglove_bridge matches the whole topic name (std::regex_match).
    return any(re.fullmatch(p, topic) for p in patterns)


# Published by rover_drive_interface. The UI's foxglove client takes a publisher's message
# definition from the server channel with the same name, so a publish-only topic missing here
# makes it wait forever and send nothing: manual driving was silently dead on 2026-09-26.
@pytest.mark.parametrize('topic', [
    '/rover/teleop_driver_interface_cmd_vel_stamped',
    '/rover/initialpose',
])
def test_topics_the_drive_ui_publishes_are_advertised(patterns, topic):
    assert _allowed(patterns, topic)


@pytest.mark.parametrize('topic', [
    '/tf', '/tf_static', '/rover/map', '/rover/scan', '/rover/plan',
    '/rover/global_costmap/costmap', '/rover/diagnostics_agg', '/rover/motion_lock',
    '/rover/hardware_interface/safety_status', '/rover/rover_battery/battery_status',
    '/rover/led/channel_1_preview', '/rover/rc/channels', '/rover/rc/calibration/state',
])
def test_topics_the_uis_subscribe_to_are_advertised(patterns, topic):
    assert _allowed(patterns, topic)


def test_namespace_is_not_hard_coded(patterns):
    assert _allowed(patterns, '/rover_a2/scan')
    assert _allowed(patterns, '/scan')


@pytest.mark.parametrize('topic', [
    # Full-rate or internal topics a browser must not pull through the router.
    '/rover/led/channel_1_frame', '/rover/rslidar_points', '/rover/diagnostics', '/rosout',
    '/rover/cmd_vel', '/rover/teleop_driver_interface_cmd_vel_fresh_stamped',
    '/rover/odometry/wheels', '/rover/joint_states',
])
def test_internal_topics_stay_off_the_bridge(patterns, topic):
    assert not _allowed(patterns, topic)
