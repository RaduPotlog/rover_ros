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

#ifndef ROVER_HARDWARE_INTERFACE_ROVER_CONTROLLER_SYSTEM_E_STOP_SYSTEM_E_STOP_HPP_
#define ROVER_HARDWARE_INTERFACE_ROVER_CONTROLLER_SYSTEM_E_STOP_SYSTEM_E_STOP_HPP_

#include <atomic>
#include <memory>
#include <mutex>

#include "rover_hardware_interface/rover_controller/rover_controller.hpp"
#include "rover_hardware_interface/rover_driver/rover_driver.hpp"

namespace rover_hardware_interface
{

class EmergencyStopInterface
{

public:

    EmergencyStopInterface() {}

    virtual ~EmergencyStopInterface() = default;

    virtual bool readEStopState() = 0;
    
    virtual bool readEStopLatchState() = 0;

    virtual void setEStop() = 0;
    
    virtual void resetEStop() = 0;

    virtual void resetEStopLatch() = 0;
};

class EmergencyStop : public EmergencyStopInterface
{

public:

    EmergencyStop(
        std::shared_ptr<RoverController> rover_controller,
        std::shared_ptr<RoverDriverInterface> rover_driver,
        std::shared_ptr<std::mutex> rover_driver_write_mtx, 
        std::function<bool()> zero_velocity_check);

    virtual ~EmergencyStop() override = default;

    bool readEStopState() override;

    bool readEStopLatchState() override;

    void setEStop() override;

    void resetEStop() override;

    void resetEStopLatch() override;

protected:

    std::shared_ptr<RoverController> rover_controller_;
    std::shared_ptr<RoverDriverInterface> rover_driver_;
    std::shared_ptr<std::mutex> rover_driver_write_mtx_;

    std::function<bool()> zeroVelocityCheck;

    std::mutex e_stop_manipulation_mtx_;
    std::atomic_bool e_stop_triggered_ = true;
};

}  // namespace rover_hardware_interface

#endif  // ROVER_HARDWARE_INTERFACE_ROVER_CONTROLLER_SYSTEM_E_STOP_SYSTEM_E_STOP_HPP_
