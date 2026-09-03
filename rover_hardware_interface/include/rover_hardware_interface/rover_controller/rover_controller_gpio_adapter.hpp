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

#ifndef ROVER_HARDWARE_INTERFACE_ROVER_CONTROLLER_ROVER_CONTROLLER_GPIO_ADAPTER_HPP_
#define ROVER_HARDWARE_INTERFACE_ROVER_CONTROLLER_ROVER_CONTROLLER_GPIO_ADAPTER_HPP_

#include <memory>
#include <unordered_map>

#include "rover_hardware_interface/domain/rover_gpio_port.hpp"
#include "rover_hardware_interface/rover_controller/rover_controller.hpp"

namespace rover_hardware_interface
{

// Infrastructure adapter: implements the domain's RoverGpioPort on top of the concrete
// Modbus-backed RoverController, so RoverSystem can depend on the port instead of on
// RoverController directly (mirrors RoverControllerEStopIo for EmergencyStopIoPort).
class RoverControllerGpioAdapter : public RoverGpioPort
{

public:

    explicit RoverControllerGpioAdapter(std::shared_ptr<RoverController> rover_controller);

    void start() override;

    void eStopUserBtnTrigger(const bool state) override;

    void eStopMotorDriverFaultTrigger(const bool state) override;

    const std::unordered_map<RoverControllerGpio, bool> & queryControlInterfaceIOStates() override;

private:

    std::shared_ptr<RoverController> rover_controller_;
};

}  // namespace rover_hardware_interface

#endif  // ROVER_HARDWARE_INTERFACE_ROVER_CONTROLLER_ROVER_CONTROLLER_GPIO_ADAPTER_HPP_
