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

#include "rover_hardware_interface/domain/emergency_stop.hpp"

#include <exception>
#include <stdexcept>
#include <string>
#include <utility>

namespace rover_hardware_interface
{

EmergencyStop::EmergencyStop(
    std::shared_ptr<EmergencyStopIoPort> io,
    std::function<bool()> zero_velocity_check)
: EmergencyStopInterface()
, io_(std::move(io))
, zero_velocity_check_(std::move(zero_velocity_check))
{

}

bool EmergencyStop::readEStopState()
{
    // Polarity matches the write side (setEStop()/resetEStop() below): the port reports `true`
    // when the E-Stop is triggered, `false` when clear — isUserButtonActive() already *is*
    // "is triggered," no negation.
    if (e_stop_manipulation_mtx_.try_lock()) {
        std::lock_guard<std::mutex> e_stop_lck(e_stop_manipulation_mtx_, std::adopt_lock);
        user_e_stop_triggered_ = io_->isUserButtonActive();
    }

    return user_e_stop_triggered_;
}

bool EmergencyStop::readEStopLatchState()
{
    if (e_stop_manipulation_mtx_.try_lock()) {
        std::lock_guard<std::mutex> e_stop_lck(e_stop_manipulation_mtx_, std::adopt_lock);
        latch_triggered_ = io_->isLatchActive();
    }

    return latch_triggered_;
}

void EmergencyStop::setEStop()
{
    std::lock_guard<std::mutex> e_stop_lck(e_stop_manipulation_mtx_);

    try {
        io_->triggerUserButton(true);
    } catch (const std::exception & e) {
        // Widened from std::runtime_error: MB::ModbusException (surfaced through the Modbus IO
        // chain) derives directly from std::exception, not std::runtime_error, and must be
        // wrapped here like any other IO failure rather than passed through unwrapped.
        throw std::runtime_error("Setting User E-Stop failed: " + std::string(e.what()));
    }
}

void EmergencyStop::resetEStop()
{
    std::lock_guard<std::mutex> e_stop_lck(e_stop_manipulation_mtx_);

    if (zero_velocity_check_ && !zero_velocity_check_()) {
        throw std::runtime_error(
            "Can't reset User E-Stop: velocity commands are not zero.");
    }

    try {
        io_->triggerUserButton(false);
    } catch (const std::exception & e) {
        // Widened from std::runtime_error: see setEStop() above.
        throw std::runtime_error("Error when trying to reset User E-Stop: " + std::string(e.what()));
    }
}

void EmergencyStop::resetEStopLatch()
{
    std::lock_guard<std::mutex> e_stop_lck(e_stop_manipulation_mtx_);

    try {
        io_->resetLatch();
    } catch (const std::exception & e) {
        // Widened from std::runtime_error: see setEStop() above.
        throw std::runtime_error("Error when trying to reset E-Stop Latch: " + std::string(e.what()));
    }
}

}  // namespace rover_hardware_interface
