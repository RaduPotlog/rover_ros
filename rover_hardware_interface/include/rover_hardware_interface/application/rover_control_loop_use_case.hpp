// Copyright 2025 Mechatronics Academy
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROVER_HARDWARE_INTERFACE_APPLICATION_ROVER_CONTROL_LOOP_USE_CASE_HPP_
#define ROVER_HARDWARE_INTERFACE_APPLICATION_ROVER_CONTROL_LOOP_USE_CASE_HPP_

#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "rover_hardware_interface/domain/emergency_stop.hpp"
#include "rover_hardware_interface/domain/rover_driver.hpp"
#include "rover_hardware_interface/domain/rover_error_filter.hpp"

namespace rover_hardware_interface
{

// Outcome of a write attempt serialized through
// RoverControlLoopUseCase::performWriteOperation().
enum class WriteOperationOutcome
{
    kSucceeded,
    // A concurrent caller already held the write lock this cycle; `write_operation` was NOT
    // attempted.
    kLockContention,
    // `write_operation` threw; see WriteOperationResult::error_message.
    kExceptionThrown,
};

struct WriteOperationResult
{
    WriteOperationOutcome outcome;
    // Populated only when outcome == kExceptionThrown.
    std::string error_message;
};

// Application layer: the safety- and error-filter-relevant decisions RoverSystem's RT
// read()/write() cycle makes every tick (see rover_system.hpp), extracted so they're
// unit-testable with fake ports the same way EmergencyStop/RoverErrorFilter already are (see
// .claude/rules/clean_architecture.md). Depends only on domain ports (RoverDriverInterface,
// EmergencyStopInterface, RoverErrorFilter) - no rclcpp::Time, no logging, no ros2_control
// buffers. RoverSystem/RoverA1System keep everything that inherently needs those: `time`-stamped
// hw-state conversion (updateHwStates()), diagnostic/state-message population
// (updateDriverStateMsg()), and RCLCPP_* logging of the outcomes this class reports.
//
// RT-safety contract mirrors RoverSystem's (see rover_system.hpp §5): performWriteOperation()
// uses try_lock and never blocks; every other method here is a bounded, allocation-free call into
// already-cached port state.
class RoverControlLoopUseCase
{

public:

    RoverControlLoopUseCase(
        std::shared_ptr<RoverDriverInterface> rover_driver,
        std::shared_ptr<EmergencyStopInterface> e_stop,
        RoverErrorFilter & error_filter);

    // Reports RoverDriverInterface::isMotorStatesDataTimedOut() to the READ_MOTOR_STATES
    // error-filter category. Call after the driver-specific hw-states conversion step has run
    // for this cycle.
    void updateMotorsStateTimeoutStatus();

    // Reports RoverDriverInterface::isDriverStateDataTimedOut() to the READ_DRIVER_STATE
    // error-filter category.
    void updateDriverStateTimeoutStatus();

    // Reports RoverDriverInterface::isFlagError() to the FAULT_FLAG error-filter category, and -
    // while a fault flag is active - attempts to clear it via a serialized call to
    // RoverDriverInterface::attemptErrorFlagReset() (see performWriteOperation()). Returns the
    // outcome of that reset attempt, or std::nullopt when no fault flag is active (no reset
    // attempted).
    std::optional<WriteOperationResult> updateFaultFlagStatus();

    // Recomputes and returns whether the E-Stop (user-triggered or latched) is currently active.
    // Fail-safe: returns true when no EmergencyStopInterface was configured.
    bool updateEStopActiveState();

    // Whether write() should command motion this cycle, given the hardware component's lifecycle
    // state and the last-computed E-Stop state. Pure decision, no I/O.
    static bool shouldCommandMotion(const bool lifecycle_active, const bool e_stop_active);

    // Serializes `write_operation` against concurrent callers via try_lock (never blocks - see
    // the RT-safety contract above) and reports the outcome to the WRITE_CMDS error-filter
    // category. On contention, `write_operation` is not invoked at all.
    WriteOperationResult performWriteOperation(const std::function<void()> & write_operation);

private:

    std::shared_ptr<RoverDriverInterface> rover_driver_;
    std::shared_ptr<EmergencyStopInterface> e_stop_;
    RoverErrorFilter & error_filter_;

    // Serializes calls to performWriteOperation() - the same single mutex write() (speed command)
    // and updateFaultFlagStatus() (error-flag reset) shared before this extraction, so the two
    // remain mutually exclusive against each other exactly as before.
    std::mutex write_mtx_;
};

}  // namespace rover_hardware_interface

#endif  // ROVER_HARDWARE_INTERFACE_APPLICATION_ROVER_CONTROL_LOOP_USE_CASE_HPP_
