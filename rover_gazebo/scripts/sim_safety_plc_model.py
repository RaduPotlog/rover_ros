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

"""Pure model of the rover's safety PLC E-Stop latch, for the simulation (no ROS imports).

Mirrors rover_arch/SAFETY_CHAIN.md: a set-dominant SR latch whose SET sources are the physical
HW E-Stop button and the SW user E-Stop coil, and whose open/closed state drives the motor
contactor. Resetting the latch has no effect while any SET source is still asserted; the SW coil
can only be released while the wheels are at rest (RoverA1System's zero-velocity check).

The SW motor-driver-fault coil and the CPU watchdog are not modelled: nothing in the simulation
can trip them.
"""

from dataclasses import dataclass

# Values of rover_msgs/SafetyStatus.LATCH_CAUSE_*, duplicated so this module stays ROS-free.
LATCH_CAUSE_UNKNOWN = 0
LATCH_CAUSE_SW_USER_BUTTON = 1
LATCH_CAUSE_HW_USER_BUTTON = 4


@dataclass(frozen=True)
class TriggerResult:
    """Outcome of a std_srvs/Trigger-style request: success flag plus a reason."""

    success: bool
    message: str = ""


class SimSafetyPlc:

    def __init__(self, latch_set_at_startup: bool = False):
        self._hw_button = False
        self._sw_user_coil = False
        self._latch = latch_set_at_startup
        self._latch_cause = LATCH_CAUSE_UNKNOWN

    @property
    def hw_button(self) -> bool:
        return self._hw_button

    @property
    def sw_user_coil(self) -> bool:
        return self._sw_user_coil

    @property
    def latch_active(self) -> bool:
        return self._latch

    @property
    def latch_cause(self) -> int:
        return self._latch_cause

    @property
    def contactor_engaged(self) -> bool:
        # The latch opens the contactor; the simulation has no welded contacts.
        return not self._latch

    def set_hw_button(self, pressed: bool) -> None:
        """The maintained HW mushroom button: pressed stays pressed until released."""
        self._hw_button = pressed
        self._apply_set_sources()

    def sw_set(self) -> TriggerResult:
        self._sw_user_coil = True
        self._apply_set_sources()
        return TriggerResult(True, "SW E-Stop set")

    def sw_reset(self, wheels_stopped: bool) -> TriggerResult:
        """Release the SW user E-Stop coil. The latch it set stays set until latch_reset()."""
        if not wheels_stopped:
            # Same refusal as EmergencyStop::resetEStop() on the rover.
            return TriggerResult(False, "Can't reset User E-Stop: velocity commands are not zero.")
        self._sw_user_coil = False
        if self._latch:
            return TriggerResult(True, "SW E-Stop released; latch still set: reset the latch")
        return TriggerResult(True, "SW E-Stop released")

    def latch_reset(self) -> TriggerResult:
        """Pulse the latch reset. Set-dominant: a still-asserted SET source keeps it latched.

        Succeeds like the rover's service does (the reset pulse was sent); the message says why
        the latch did not clear.
        """
        blockers = self._active_set_sources()
        if blockers:
            return TriggerResult(True, "latch still set: " + " and ".join(blockers))
        self._latch = False
        self._latch_cause = LATCH_CAUSE_UNKNOWN
        return TriggerResult(True, "latch reset")

    def _active_set_sources(self) -> list:
        sources = []
        if self._hw_button:
            sources.append("HW E-Stop pressed")
        if self._sw_user_coil:
            sources.append("SW E-Stop set")
        return sources

    def _apply_set_sources(self) -> None:
        if self._latch:
            return
        if self._hw_button:
            self._latch = True
            self._latch_cause = LATCH_CAUSE_HW_USER_BUTTON
        elif self._sw_user_coil:
            self._latch = True
            self._latch_cause = LATCH_CAUSE_SW_USER_BUTTON
