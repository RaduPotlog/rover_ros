# Copyright 2025 Mechatronics Academy
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

"""Wheel geometry in the controller config must match rover_description.

rover_description/config/<wheel_type>.yaml is the single source of truth for
wheel geometry (it drives the URDF). diff_drive_controller needs the same
values as plain parameters, so this test fails the build if they drift.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import pytest
import yaml

CONTROLLER_CONFIG_DIR = Path(__file__).resolve().parent.parent / 'config'
WHEEL_TYPES = ['wheel_01']


def _load(path):
    with open(path, encoding='utf-8') as f:
        return yaml.safe_load(f)


@pytest.mark.parametrize('wheel_type', WHEEL_TYPES)
def test_drive_controller_matches_description(wheel_type):
    description_dir = Path(get_package_share_directory('rover_description')) / 'config'
    wheel = _load(description_dir / f'{wheel_type}.yaml')
    controller = _load(CONTROLLER_CONFIG_DIR / f'{wheel_type}_controller.yaml')

    drive = controller['/**']['rover_drive_controller']['ros__parameters']
    assert drive['wheel_radius'] == pytest.approx(wheel['wheel_radius'])
    assert drive['wheel_separation'] == pytest.approx(wheel['wheel_separation'])


@pytest.mark.parametrize('wheel_type', WHEEL_TYPES)
def test_drive_controller_chains_through_declared_wheel_pids(wheel_type):
    """Every '<pid>/<joint>' wheel name must point at a declared PID for that joint."""
    config = _load(CONTROLLER_CONFIG_DIR / f'{wheel_type}_controller.yaml')['/**']
    manager = config['controller_manager']['ros__parameters']
    drive = config['rover_drive_controller']['ros__parameters']
    wheels = drive['left_wheel_names'] + drive['right_wheel_names']
    assert len(wheels) == 2 * drive['wheels_per_side']
    for wheel in wheels:
        pid_name, joint = wheel.split('/', 1)
        assert manager[pid_name]['type'] == 'pid_controller/PidController'
        pid = config[pid_name]['ros__parameters']
        assert pid['dof_names'] == [joint]
        assert pid['command_interface'] == 'velocity'
        assert pid['reference_and_state_interfaces'] == ['velocity']
        # Feed-forward carries the open-loop command; PI only trims it.
        assert pid['gains'][joint]['feedforward_gain'] == pytest.approx(1.0)


@pytest.mark.parametrize('wheel_type', WHEEL_TYPES)
def test_calibration_multipliers_are_plausible(wheel_type):
    """Skid-steer separation multipliers are > 1; radius multipliers stay near 1."""
    drive = _load(CONTROLLER_CONFIG_DIR / f'{wheel_type}_controller.yaml')[
        '/**']['rover_drive_controller']['ros__parameters']
    assert 1.0 <= drive['wheel_separation_multiplier'] <= 2.5
    for side in ('left', 'right'):
        assert 0.9 <= drive[f'{side}_wheel_radius_multiplier'] <= 1.1
