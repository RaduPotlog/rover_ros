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
import re

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
        # Seeded, not the stock PidController: see seeded_pid_controller.hpp.
        assert manager[pid_name]['type'] == 'rover_controller/SeededPidController'
        pid = config[pid_name]['ros__parameters']
        assert pid['dof_names'] == [joint]
        assert pid['command_interface'] == 'velocity'
        assert pid['reference_and_state_interfaces'] == ['velocity']
        # Feed-forward carries the open-loop command; PI only trims it.
        assert pid['gains'][joint]['feedforward_gain'] == pytest.approx(1.0)
        # Integral is cleared on (re)activation - no stale I-term after an e-stop.
        assert pid['gains'][joint]['save_i_term'] is False


@pytest.mark.parametrize('wheel_type', WHEEL_TYPES)
def test_calibration_multipliers_are_plausible(wheel_type):
    """Skid-steer separation multipliers are > 1; radius multipliers stay near 1."""
    drive = _load(CONTROLLER_CONFIG_DIR / f'{wheel_type}_controller.yaml')[
        '/**']['rover_drive_controller']['ros__parameters']
    assert 1.0 <= drive['wheel_separation_multiplier'] <= 2.5
    for side in ('left', 'right'):
        assert 0.9 <= drive[f'{side}_wheel_radius_multiplier'] <= 1.1


def _urdf_wheel_velocity_limit():
    xacro = (Path(get_package_share_directory('rover_description'))
             / 'urdf' / 'common' / 'wheel.urdf.xacro').read_text(encoding='utf-8')
    match = re.search(r'<limit\b[^>]*\bvelocity="([0-9.]+)"', xacro)
    assert match, 'wheel joint <limit velocity="..."> not found in wheel.urdf.xacro'
    return float(match.group(1))


@pytest.mark.parametrize('wheel_type', WHEEL_TYPES)
def test_drive_limits_respect_joint_velocity_limit(wheel_type):
    """Full linear + full angular must keep the outer wheel under the URDF limit.

    diff_drive limits linear.x and angular.z independently, and each wheel PID adds up to
    i_clamp_max (plus a small P term) on top of its feed-forward reference, outside u_clamp.
    """
    config = _load(CONTROLLER_CONFIG_DIR / f'{wheel_type}_controller.yaml')['/**']
    drive = config['rover_drive_controller']['ros__parameters']
    half_track = drive['wheel_separation'] * drive['wheel_separation_multiplier'] / 2.0
    max_v = max(drive['linear']['x']['max_velocity'], -drive['linear']['x']['min_velocity'])
    max_w = max(drive['angular']['z']['max_velocity'], -drive['angular']['z']['min_velocity'])
    wheel_reference = (max_v + max_w * half_track) / drive['wheel_radius']

    limit = _urdf_wheel_velocity_limit()
    for wheel in drive['left_wheel_names'] + drive['right_wheel_names']:
        pid_name, joint = wheel.split('/', 1)
        i_clamp = config[pid_name]['ros__parameters']['gains'][joint]['i_clamp_max']
        assert wheel_reference + i_clamp <= limit, (
            f'{joint}: outer wheel reference {wheel_reference:.2f} rad/s + I {i_clamp} '
            f'exceeds the URDF limit {limit} rad/s')


@pytest.mark.parametrize('wheel_type', WHEEL_TYPES)
def test_controller_update_rates_divide_manager_rate(wheel_type):
    """A per-controller update_rate must divide the controller_manager rate evenly.

    Otherwise controller_manager still runs it, but at an uneven period, and warns once.
    """
    config = _load(CONTROLLER_CONFIG_DIR / f'{wheel_type}_controller.yaml')['/**']
    manager = config['controller_manager']['ros__parameters']
    manager_rate = manager['update_rate']
    controllers = [name for name, value in manager.items()
                   if isinstance(value, dict) and 'type' in value]
    for name in controllers:
        rate = config.get(name, {}).get('ros__parameters', {}).get('update_rate')
        if rate is None:
            continue
        assert 0 < rate <= manager_rate, f'{name}: update_rate {rate} Hz'
        assert manager_rate % rate == 0, (
            f'{name}: update_rate {rate} Hz does not divide {manager_rate} Hz')
