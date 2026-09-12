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

The second test pins the REP-103 frame convention itself: +x forward, +y left,
so fl/fr/rl/rr must land in the (+x,+y)/(+x,-y)/(-x,+y)/(-x,-y) quadrants at
half the (full) wheelbase and wheel_separation.
"""

from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import pytest
import xacro
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


# REP-103: +x forward, +y left. Signs of (x, y) per wheel prefix.
WHEEL_QUADRANTS = {
    'fl': (+1, +1),
    'fr': (+1, -1),
    'rl': (-1, +1),
    'rr': (-1, -1),
}


@pytest.mark.parametrize('wheel_type', WHEEL_TYPES)
def test_urdf_wheel_placement_matches_convention(wheel_type):
    """Each wheel mount sits in its named quadrant, at half the config distances."""
    description_dir = Path(get_package_share_directory('rover_description'))
    wheel_config = description_dir / 'config' / f'{wheel_type}.yaml'
    wheel = _load(wheel_config)

    doc = xacro.process_file(
        str(description_dir / 'urdf' / 'rover_a1.urdf.xacro'),
        mappings={'wheel_config_file': str(wheel_config)},
    )
    root = ET.fromstring(doc.toxml())
    origins = {
        joint.get('name'): [float(v) for v in joint.find('origin').get('xyz').split()]
        for joint in root.findall('joint')
        if joint.find('origin') is not None
    }

    half_x = wheel['wheelbase'] / 2.0
    half_y = wheel['wheel_separation'] / 2.0

    for prefix, (sx, sy) in WHEEL_QUADRANTS.items():
        x, y, z = origins[f'body_to_{prefix}_wheel_base_joint']
        assert x == pytest.approx(sx * half_x), f'{prefix} wheel x'
        assert y == pytest.approx(sy * half_y), f'{prefix} wheel y'

        # All four wheels mount at the same height.
        assert z == pytest.approx(origins['body_to_fl_wheel_base_joint'][2])

    # base_footprint is the ground projection: wheel axis height - wheel radius.
    axis_z = origins['body_to_fl_wheel_base_joint'][2]
    footprint_z = origins['body_to_footprint_joint'][2]
    assert footprint_z == pytest.approx(axis_z - wheel['wheel_radius'])
