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

#include <chrono>
#include <cstdio>
#include <memory>
#include <string>

#include "rover_hardware_interface/system_e_stop/system_e_stop.hpp"

namespace rover_hardware_interface
{

EmergencyStop::EmergencyStop(
    std::shared_ptr<RoverController> rover_controller,
    std::shared_ptr<RoverDriverInterface> rover_driver,
    std::shared_ptr<std::mutex> rover_driver_write_mtx, 
    std::function<bool()> zero_velocity_check)
: EmergencyStopInterface()
, rover_controller_(rover_controller)
, rover_driver_(rover_driver)
, rover_driver_write_mtx_(rover_driver_write_mtx)
, zeroVelocityCheck(zero_velocity_check) 
{

};

bool EmergencyStop::readEStopState()
{
    if (e_stop_manipulation_mtx_.try_lock()) {
        std::lock_guard<std::mutex> e_stop_lck(e_stop_manipulation_mtx_, std::adopt_lock);
        e_stop_triggered_ = !rover_controller_->isPinActive(RoverControllerGpio::GPIO_SW_E_STOP_USER_BUTTON);
    }

    return e_stop_triggered_;
}

bool EmergencyStop::readEStopLatchState()
{
    if (e_stop_manipulation_mtx_.try_lock()) {
        std::lock_guard<std::mutex> e_stop_lck(e_stop_manipulation_mtx_, std::adopt_lock);
        e_stop_triggered_ = !rover_controller_->isPinActive(RoverControllerGpio::GPIO_SW_E_STOP_LATCH_STATUS);
    }

    return e_stop_triggered_;
}

void EmergencyStop::setEStop()
{
    std::lock_guard<std::mutex> e_stop_lck(e_stop_manipulation_mtx_);
    
    try {
        rover_controller_->eStopUserBtnTrigger(true);
    } catch (const std::runtime_error & e) {
        throw std::runtime_error("Setting User E-Stop failed: " + std::string(e.what()));
    }
}

void EmergencyStop::resetEStop()
{
    std::lock_guard<std::mutex> e_stop_lck(e_stop_manipulation_mtx_);
    
    try {
        rover_controller_->eStopUserBtnTrigger(false);
    } catch (const std::runtime_error & e) {
        throw std::runtime_error("Error when trying to reset User E-Stop: " + std::string(e.what()));
    }
}

void EmergencyStop::resetEStopLatch()
{
    std::lock_guard<std::mutex> e_stop_lck(e_stop_manipulation_mtx_);
    
    try {
        rover_controller_->eStopLatchReset();
    } catch (const std::runtime_error & e) {
        throw std::runtime_error("Error when trying to reset E-Stop Latch: " + std::string(e.what()));
    }
}

}  // namespace namespace rover_hardware_interface

