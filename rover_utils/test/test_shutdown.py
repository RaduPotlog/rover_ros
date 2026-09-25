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

from types import SimpleNamespace

from launch.actions import Shutdown
import pytest

from rover_utils.shutdown import shutdown_unless_shutting_down


# Any exit of a required process ends the launch: a crash, a clean exit and a signal alike.
@pytest.mark.parametrize('returncode', [1, 0, -9])
def test_a_process_exiting_on_its_own_shuts_the_launch_down(returncode):
    on_exit = shutdown_unless_shutting_down('ros2_control_node')
    actions = on_exit(SimpleNamespace(returncode=returncode), SimpleNamespace(is_shutdown=False))
    assert len(actions) == 1
    assert isinstance(actions[0], Shutdown)
    assert actions[0].event.reason == f'ros2_control_node exited with code {returncode}'


# A second Shutdown during teardown makes launch_ros shut its ROS adapter down twice.
@pytest.mark.parametrize('returncode', [1, 0])
def test_an_exit_during_shutdown_emits_nothing(returncode):
    on_exit = shutdown_unless_shutting_down('rover_led_container')
    actions = on_exit(SimpleNamespace(returncode=returncode), SimpleNamespace(is_shutdown=True))
    assert actions == []
