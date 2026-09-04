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

#include "rover_hardware_interface/application/rover_control_loop_use_case.hpp"

#include <exception>
#include <mutex>
#include <utility>

namespace rover_hardware_interface
{

RoverControlLoopUseCase::RoverControlLoopUseCase(
    std::shared_ptr<RoverDriverInterface> rover_driver,
    std::shared_ptr<EmergencyStopInterface> e_stop,
    RoverErrorFilter & error_filter)
: rover_driver_(std::move(rover_driver))
, e_stop_(std::move(e_stop))
, error_filter_(error_filter)
{

}

void RoverControlLoopUseCase::updateMotorsStateTimeoutStatus()
{
    error_filter_.updateError(
        ErrorsFilterIds::READ_MOTOR_STATES, rover_driver_->isMotorStatesDataTimedOut());
}

void RoverControlLoopUseCase::updateDriverStateTimeoutStatus()
{
    error_filter_.updateError(
        ErrorsFilterIds::READ_DRIVER_STATE, rover_driver_->isDriverStateDataTimedOut());
}

std::optional<WriteOperationResult> RoverControlLoopUseCase::updateFaultFlagStatus()
{
    const bool flag_error = rover_driver_->isFlagError();
    error_filter_.updateError(ErrorsFilterIds::FAULT_FLAG, flag_error);

    if (!flag_error) {
        return std::nullopt;
    }

    return performWriteOperation([this] { rover_driver_->attemptErrorFlagReset(); });
}

bool RoverControlLoopUseCase::updateEStopActiveState()
{
    if (!e_stop_) {
        return true;
    }

    const bool user_e_stop_triggered = e_stop_->readEStopState();
    const bool e_stop_latched = e_stop_->readEStopLatchState();

    return user_e_stop_triggered || e_stop_latched;
}

bool RoverControlLoopUseCase::shouldCommandMotion(const bool lifecycle_active, const bool e_stop_active)
{
    return lifecycle_active && !e_stop_active;
}

WriteOperationResult RoverControlLoopUseCase::performWriteOperation(
    const std::function<void()> & write_operation)
{
    std::unique_lock<std::mutex> write_lck(write_mtx_, std::defer_lock);

    if (!write_lck.try_lock()) {
        error_filter_.updateError(ErrorsFilterIds::WRITE_CMDS, true);
        return {WriteOperationOutcome::kLockContention, ""};
    }

    try {
        write_operation();
        error_filter_.updateError(ErrorsFilterIds::WRITE_CMDS, false);
        return {WriteOperationOutcome::kSucceeded, ""};
    } catch (const std::exception & e) {
        // Copies e.what() into error_message on every exception, unlike the old inline
        // RCLCPP_WARN_STREAM_THROTTLE call this replaced, which only paid its string-building
        // cost on ticks the 5s throttle let through - the caller (RoverSystem::
        // logWriteOperationResult()) now owns that throttling instead. Accepted: the exception
        // itself (__cxa_allocate_exception et al.) already costs far more than one std::string
        // copy, so this doesn't change the RT-budget picture on the failure path.
        error_filter_.updateError(ErrorsFilterIds::WRITE_CMDS, true);
        return {WriteOperationOutcome::kExceptionThrown, e.what()};
    }
}

}  // namespace rover_hardware_interface
