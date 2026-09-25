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

import pytest

from rover_utils.version_check import check_version_compatibility, extract_version_tuple


@pytest.mark.parametrize(
    'version_string, expected',
    [('v1.2.3', (1, 2, 3)), ('v0.0.0', (0, 0, 0)), ('v10.20.30', (10, 20, 30)),
     ('v01.002.0003', (1, 2, 3))],
)
def test_extract_version_tuple_reads_vx_y_z(version_string, expected):
    assert extract_version_tuple(version_string) == expected


@pytest.mark.parametrize(
    'version_string, expected',
    [
        # Unanchored search: the first vX.Y.Z anywhere in the string wins.
        ('rover-os-v2.4.1', (2, 4, 1)),
        ('  v1.2.3\n', (1, 2, 3)),
        ('vv1.2.3', (1, 2, 3)),
        ('v1.2.3 v4.5.6', (1, 2, 3)),
        # Anything after the patch number is ignored: pre-release, build metadata, a 4th field.
        ('v1.2.3-rc1', (1, 2, 3)),
        ('v1.0.0-beta.2+build.5', (1, 0, 0)),
        ('v1.2.3.4', (1, 2, 3)),
    ],
)
def test_extract_version_tuple_takes_the_first_vx_y_z_and_ignores_the_rest(
        version_string, expected):
    assert extract_version_tuple(version_string) == expected


@pytest.mark.parametrize(
    'version_string',
    ['', '1.2.3', 'release-1.2.3', 'V1.2.3', 'v1.2', 'v1.2.x', 'v 1.2.3', 'v1..2.3', 'v-1.2.3'],
)
def test_extract_version_tuple_is_v0_0_0_without_a_vx_y_z(version_string):
    assert extract_version_tuple(version_string) == (0, 0, 0)


# `is expected`, not `==`: rover_bringup pastes the result into a PythonExpression, so it must
# be a real bool ("True"/"False").
@pytest.mark.parametrize(
    'version, min_required, expected',
    [
        ('v1.0.0', 'v1.0.0', True),  # rover_bringup's default SYSTEM_BUILD_VERSION and minimum
        ('v1.0.1', 'v1.0.0', True),
        ('v1.0.0', 'v1.0.1', False),
        ('v1.1.0', 'v1.0.9', True),
        ('v1.0.9', 'v1.1.0', False),
        ('v2.0.0', 'v1.99.99', True),
        ('v1.99.99', 'v2.0.0', False),
        ('v1.10.0', 'v1.9.0', True),  # numbers, not strings
        ('v1.9.0', 'v1.10.0', False),
    ],
)
def test_check_version_compatibility_compares_major_minor_patch_as_numbers(
        version, min_required, expected):
    assert check_version_compatibility(version, min_required) is expected


@pytest.mark.parametrize(
    'version, min_required, expected',
    [('v1.0.0-rc1', 'v1.0.0', True), ('v0.9.9-rc1', 'v1.0.0', False)],
)
def test_check_version_compatibility_counts_a_pre_release_as_its_release(
        version, min_required, expected):
    assert check_version_compatibility(version, min_required) is expected


@pytest.mark.parametrize(
    'version, min_required, expected',
    [
        # An unparsable version is v0.0.0: it fails a real minimum, so a malformed
        # SYSTEM_BUILD_VERSION makes rover_bringup warn ...
        ('1.2.3', 'v1.0.0', False),
        ('', 'v1.0.0', False),
        # ... and passes a v0.0.0 minimum.
        ('garbage', 'v0.0.0', True),
        # An unparsable minimum is v0.0.0 too, so everything passes it: it needs its 'v'.
        ('v0.0.1', '1.0.0', True),
        ('v0.0.0', 'nonsense', True),
        ('', '', True),
    ],
)
def test_check_version_compatibility_treats_an_unparsable_string_as_v0_0_0(
        version, min_required, expected):
    assert check_version_compatibility(version, min_required) is expected
