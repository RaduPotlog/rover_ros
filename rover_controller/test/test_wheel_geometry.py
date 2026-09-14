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

    drive = controller['/**']['drive_controller']['ros__parameters']
    assert drive['wheel_radius'] == pytest.approx(wheel['wheel_radius'])
    assert drive['wheel_separation'] == pytest.approx(wheel['wheel_separation'])
