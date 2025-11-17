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

namespace rover_hardware_interface
{

RoverModbus::RoverModbus(const std::string &ip, const int port)
{
    if (ip.empty()) {
        throw std::invalid_argument("Please provide an IP address for TCP connection.");
    }

    // TODO: re-try defined by a config
    while (true) {
        try {
            connection_ = std::make_unique<ModbusTcpConnection>(ip, port);
            break;
        } catch (const std::exception &ex) {
            std::cerr << "Failed to establish TCP connection: " << ex.what() << ". Retrying in 1 second..." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
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
        std::cerr << "Failed to read contact" << std::endl;
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
        std::cerr << "Failed to read coil" << std::endl;
        throw;
    }

    return coil_value;
}

void RoverModbus::writeDiscreteCoil(const CoilInfo &coil, const bool coil_state)
{
    if (!coil.is_coil_engage_allowed) {
        std::cerr << "Coil engage is not allowed" << std::endl;
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
        std::cerr << "Failed to set motor power enable" << std::endl;
        throw;
    }
}

MB::ModbusResponse RoverModbus::sendRequest(const MB::ModbusRequest &request)
{
    try {
        return connection_->sendRequest(request);
    }
    catch (const MB::ModbusException &ex) {
        std::cerr << "Modbus exception: " << ex.what() << std::endl;
        throw;
    }
}

}  // namespace rover_hardware_interface