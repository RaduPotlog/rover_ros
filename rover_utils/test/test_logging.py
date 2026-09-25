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

from launch import LaunchContext
from launch.substitutions import LaunchConfiguration
import pytest

from rover_utils.logging import limit_log_level_to_info, quiet_rmw_zenoh


def perform(substitution, log_level):
    context = LaunchContext()
    context.launch_configurations['log_level'] = log_level
    return substitution.perform(context)


@pytest.mark.parametrize(
    'log_level, expected',
    [
        ('DEBUG', 'rcl:=INFO'),
        ('debug', 'rcl:=INFO'),
        ('INFO', 'rcl:=INFO'),
        ('warn', 'rcl:=WARN'),
        ('WARNING', 'rcl:=WARN'),
        ('ERROR', 'rcl:=ERROR'),
        ('FATAL', 'rcl:=FATAL'),
    ],
)
def test_limit_log_level_to_info_caps_debug_and_passes_the_rest(log_level, expected):
    level = LaunchConfiguration('log_level')
    assert perform(limit_log_level_to_info('rcl', level), log_level) == expected


def test_limit_log_level_to_info_keeps_dotted_logger_names():
    level = LaunchConfiguration('log_level')
    result = perform(limit_log_level_to_info('pluginlib.ClassLoader', level), 'ERROR')
    assert result == 'pluginlib.ClassLoader:=ERROR'


@pytest.mark.parametrize(
    'log_level, expected',
    [
        ('DEBUG', 'DEBUG'),
        ('debug', 'debug'),
        ('INFO', 'ERROR'),
        ('WARN', 'ERROR'),
        ('ERROR', 'ERROR'),
        ('FATAL', 'FATAL'),
    ],
)
def test_quiet_rmw_zenoh(log_level, expected):
    level = LaunchConfiguration('log_level')
    assert perform(quiet_rmw_zenoh(level), log_level) == f'rmw_zenoh_cpp:={expected}'
