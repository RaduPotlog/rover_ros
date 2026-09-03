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

#include "rover_hardware_interface/rover_modbus/modbus.hpp"

#include <limits>

#include "rover_hardware_interface/utils.hpp"

namespace rover_hardware_interface
{

RoverModbus::RoverModbus(
    const std::string &ip, const int port,
    const unsigned max_connection_attempts,
    const std::chrono::milliseconds retry_delay)
{
    if (ip.empty()) {
        throw std::invalid_argument("Please provide an IP address for TCP connection.");
    }

    // A configured attempt count of 0 means "retry forever" (matches the previous unconditional
    // while(true) behavior); translated locally rather than teaching operationWithAttempts a
    // second meaning for 0 that its other callers (RoverSystem::on_activate/on_configure) don't
    // expect ("0 attempts" there means "don't even try").
    const unsigned attempts = (max_connection_attempts == 0)
        ? std::numeric_limits<unsigned>::max()
        : max_connection_attempts;

    const bool connected = operationWithAttempts(
        [this, &ip, port]() { connection_ = std::make_unique<ModbusTcpConnection>(ip, port); },
        attempts,
        []() {},
        retry_delay);

    if (!connected) {
        throw std::runtime_error(
            "Failed to establish Modbus TCP connection to " + ip + ":" + std::to_string(port) +
            " after " + std::to_string(max_connection_attempts) + " attempts.");
    }
}

RoverModbus::~RoverModbus()
{
    if (connection_) {
        connection_->close();
    }
}

uint16_t RoverModbus::readDiscreteContact(const ContactInfo &contact)
{
    uint16_t contact_value = 255U;

    MB::ModbusRequest request(MODBUS_DEVICE_ID, MB::utils::ReadDiscreteInputContacts, static_cast<uint16_t>(contact.contact), 1);

    try {
        MB::ModbusResponse resp = sendRequest(request);
        contact_value = resp.registerValues().front().isCoil() ? resp.registerValues().front().coil() : 255U;
    } catch (const MB::ModbusException &) {
        RCLCPP_ERROR(logger_, "Failed to read contact");
        throw;
    }

    return contact_value;
}

uint16_t RoverModbus::readDiscreteCoil(const CoilInfo &coil)
{
    uint16_t coil_value = 255U;

    MB::ModbusRequest request(MODBUS_DEVICE_ID, MB::utils::ReadDiscreteOutputCoils, static_cast<uint16_t>(coil.coil), 1);

    try {
        MB::ModbusResponse resp = sendRequest(request);
        coil_value = resp.registerValues().front().isCoil() ? resp.registerValues().front().coil() : 255U;
    } catch (const MB::ModbusException &){
        RCLCPP_ERROR(logger_, "Failed to read coil");
        throw;
    }

    return coil_value;
}

void RoverModbus::writeDiscreteCoil(const CoilInfo &coil, const bool coil_state)
{
    if (!coil.is_coil_engage_allowed) {
        RCLCPP_ERROR(logger_, "Coil engage is not allowed");
        return;
    }

    std::vector<MB::ModbusCell> value = {
        MB::ModbusCell(static_cast<bool>(coil_state)),
    };

    MB::ModbusRequest req(MODBUS_DEVICE_ID, MB::utils::WriteSingleDiscreteOutputCoil, static_cast<uint16_t>(coil.coil), 1, value);

    try {
        (void)sendRequest(req);
    }
    catch (const MB::ModbusException &) {
        RCLCPP_ERROR(logger_, "Failed to set motor power enable");
        throw;
    }
}

MB::ModbusResponse RoverModbus::sendRequest(const MB::ModbusRequest &request)
{
    try {
        return connection_->sendRequest(request);
    }
    catch (const MB::ModbusException &ex) {
        RCLCPP_ERROR_STREAM(logger_, "Modbus exception: " << ex.what());
        throw;
    }
}

}  // namespace rover_hardware_interface
