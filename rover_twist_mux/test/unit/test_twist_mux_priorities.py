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

"""The shipped twist_mux priorities keep the arbitration order the README documents.

twist_mux does the arbitrating; this package only hands it numbers. The safety argument (the RC
operator with line of sight has the last word, every teleop source preempts Nav 2, an E-Stop
stops everything) rests on the relative order of the priorities in config/rover_twist_mux.yaml.
Renumbering an input for an unrelated reason could invert it.

twist_mux behaviour relied on: priorities are clamped to [0, 255]; an input at priority <= 0 is
never selected; the highest-priority unexpired input wins, and of two with equal priority the
one listed first does; an input is masked while an active lock's priority is strictly above
its own.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import pytest
import yaml

# Lowest to highest; each must strictly outrank the one before it.
DOCUMENTED_ORDER = ['nav', 'driver_interface', 'joystick', 'cmd_elrs']
RC_INPUT = 'cmd_elrs'
AUTONOMY_INPUT = 'nav'
SAFETY_LOCK = 'motion_lock'
MIN_PRIORITY = 1    # twist_mux never selects an input at 0
MAX_PRIORITY = 255  # twist_mux clamps above this


@pytest.fixture(scope='module')
def mux_params():
    """Load the file rover_twist_mux.launch.py uses by default (twist_mux_config_path)."""
    path = (Path(get_package_share_directory('rover_twist_mux'))
            / 'config' / 'rover_twist_mux.yaml')
    with open(path, encoding='utf-8') as stream:
        return yaml.safe_load(stream)['/**']['ros__parameters']


@pytest.fixture(scope='module')
def inputs(mux_params):
    return {name: entry['priority'] for name, entry in mux_params['topics'].items()}


@pytest.fixture(scope='module')
def lock_priority(mux_params):
    return mux_params['locks'][SAFETY_LOCK]['priority']


def test_documented_inputs_rank_nav_driver_interface_joystick_cmd_elrs(inputs):
    missing = [name for name in DOCUMENTED_ORDER if name not in inputs]
    assert not missing, f'inputs missing from rover_twist_mux.yaml: {missing}'
    for lower, higher in zip(DOCUMENTED_ORDER, DOCUMENTED_ORDER[1:]):
        assert inputs[lower] < inputs[higher], (
            f'{lower} ({inputs[lower]}) must rank below {higher} ({inputs[higher]})')


def test_rc_transmitter_outranks_every_other_input(inputs):
    rc = inputs[RC_INPUT]
    above = {n: p for n, p in inputs.items() if n != RC_INPUT and p >= rc}
    assert not above, f'inputs at or above {RC_INPUT} ({rc}): {above}'


def test_every_other_input_outranks_nav(inputs):
    nav = inputs[AUTONOMY_INPUT]
    below = {n: p for n, p in inputs.items() if n != AUTONOMY_INPUT and p <= nav}
    assert not below, f'inputs at or below {AUTONOMY_INPUT} ({nav}): {below}'


def test_no_two_inputs_share_a_priority(inputs):
    by_priority = {}
    for name, priority in inputs.items():
        by_priority.setdefault(priority, []).append(name)
    shared = {p: names for p, names in by_priority.items() if len(names) > 1}
    assert not shared, f'twist_mux would break these ties by list order: {shared}'


def test_priorities_are_inside_the_range_twist_mux_honours(inputs, lock_priority):
    for name, priority in {**inputs, SAFETY_LOCK: lock_priority}.items():
        assert MIN_PRIORITY <= priority <= MAX_PRIORITY, (
            f'{name} priority {priority} is outside [{MIN_PRIORITY}, {MAX_PRIORITY}]')


def test_motion_lock_masks_every_input(inputs, lock_priority):
    unmasked = {n: p for n, p in inputs.items() if p >= lock_priority}
    assert not unmasked, (
        f'an asserted {SAFETY_LOCK} (priority {lock_priority}) would not stop: {unmasked}')
