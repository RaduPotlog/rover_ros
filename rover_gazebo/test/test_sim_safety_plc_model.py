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

from sim_safety_plc_model import (
    LATCH_CAUSE_HW_USER_BUTTON,
    LATCH_CAUSE_SW_USER_BUTTON,
    LATCH_CAUSE_UNKNOWN,
    SimSafetyPlc,
)


def test_starts_clear_by_default():
    plc = SimSafetyPlc()
    assert not plc.latch_active
    assert plc.contactor_engaged
    assert not plc.hw_button
    assert not plc.sw_user_coil


def test_latch_set_at_startup_needs_a_reset():
    plc = SimSafetyPlc(latch_set_at_startup=True)
    assert plc.latch_active
    assert not plc.contactor_engaged
    assert plc.latch_cause == LATCH_CAUSE_UNKNOWN
    assert plc.latch_reset().success
    assert not plc.latch_active
    assert plc.contactor_engaged


def test_hw_button_latches_and_opens_the_contactor():
    plc = SimSafetyPlc()
    plc.set_hw_button(True)
    assert plc.latch_active
    assert not plc.contactor_engaged
    assert plc.latch_cause == LATCH_CAUSE_HW_USER_BUTTON


def test_latch_reset_has_no_effect_while_hw_button_pressed():
    plc = SimSafetyPlc()
    plc.set_hw_button(True)
    result = plc.latch_reset()
    assert result.success  # the pulse is sent, as on the rover
    assert "HW E-Stop pressed" in result.message
    assert plc.latch_active


def test_releasing_hw_button_resets_the_latch():
    plc = SimSafetyPlc()
    plc.set_hw_button(True)
    result = plc.set_hw_button(False)
    assert result.message == "HW E-Stop released, latch reset"
    assert not plc.latch_active
    assert plc.contactor_engaged
    assert plc.latch_cause == LATCH_CAUSE_UNKNOWN


def test_releasing_hw_button_keeps_the_latch_while_sw_e_stop_set():
    plc = SimSafetyPlc()
    plc.sw_set()
    plc.set_hw_button(True)
    result = plc.set_hw_button(False)
    assert "SW E-Stop set" in result.message
    assert plc.latch_active


def test_releasing_hw_button_clears_a_latch_left_by_a_released_sw_e_stop():
    plc = SimSafetyPlc()
    plc.sw_set()
    plc.sw_reset(wheels_stopped=True)
    plc.set_hw_button(True)
    plc.set_hw_button(False)
    assert not plc.latch_active


def test_repeated_released_state_is_not_a_reset():
    # The panel republishes the released state every second; only the release edge resets.
    plc = SimSafetyPlc()
    plc.sw_set()
    plc.sw_reset(wheels_stopped=True)
    assert plc.set_hw_button(False).message == "HW E-Stop released"
    assert plc.latch_active


def test_sw_set_latches_and_is_echoed():
    plc = SimSafetyPlc()
    assert plc.sw_set().success
    assert plc.sw_user_coil
    assert plc.latch_active
    assert plc.latch_cause == LATCH_CAUSE_SW_USER_BUTTON


def test_sw_reset_refused_while_wheels_move():
    plc = SimSafetyPlc()
    plc.sw_set()
    result = plc.sw_reset(wheels_stopped=False)
    assert not result.success
    assert "not zero" in result.message
    assert plc.sw_user_coil


def test_sw_reset_alone_leaves_the_latch_set():
    plc = SimSafetyPlc()
    plc.sw_set()
    result = plc.sw_reset(wheels_stopped=True)
    assert result.success
    assert "latch still set" in result.message
    assert not plc.sw_user_coil
    assert plc.latch_active


def test_latch_reset_refused_while_sw_coil_set():
    plc = SimSafetyPlc()
    plc.sw_set()
    result = plc.latch_reset()
    assert "SW E-Stop set" in result.message
    assert plc.latch_active


def test_sw_reset_then_latch_reset_clears():
    plc = SimSafetyPlc()
    plc.sw_set()
    plc.sw_reset(wheels_stopped=True)
    assert plc.latch_reset().message == "latch reset"
    assert not plc.latch_active
    assert plc.contactor_engaged


def test_first_cause_is_kept():
    plc = SimSafetyPlc()
    plc.sw_set()
    plc.set_hw_button(True)
    assert plc.latch_cause == LATCH_CAUSE_SW_USER_BUTTON
