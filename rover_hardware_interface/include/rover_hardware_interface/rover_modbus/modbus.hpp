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

#ifndef ROVER_HARDWARE_INTERFACE_ROVER_MODBUS_ROVER_MODBUS_HPP_
#define ROVER_HARDWARE_INTERFACE_ROVER_MODBUS_ROVER_MODBUS_HPP_

#include <memory>
#include <vector>
#include <string>
#include <stdexcept>
#include <iostream>
#include <thread>

#include <modbusRequest.hpp>
#include <modbusResponse.hpp>
#include <modbusException.hpp>
#include <modbusUtils.hpp>

#include "rover_hardware_interface/rover_modbus/modbus_types.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>

#include "rover_hardware_interface/rover_modbus/modbus_connection.hpp"
#include "rover_hardware_interface/rover_modbus/modbus_tcp_connection.hpp"

namespace rover_hardware_interface
{

class RoverModbus
{

public:
    
    const uint8_t MODBUS_DEVICE_ID = 255U;

    RoverModbus(const std::string &ip, const int port);
    
    ~RoverModbus();

    uint16_t readDiscreteContact(const ContactInfo &contact);

    uint16_t readDiscreteCoil(const CoilInfo &coil);
    
    void writeDiscreteCoil(const CoilInfo &coil, const bool coil_state);

private:

    MB::ModbusResponse sendRequest(const MB::ModbusRequest &request);

    std::unique_ptr<ModbusConnection> connection_;

    rclcpp::Logger logger_{rclcpp::get_logger("RoverSystem")};
};

}  // namespace rover_hardware_interface

#endif  // ROVER_HARDWARE_INTERFACE_ROVER_MODBUS_ROVER_MODBUS_HPP_