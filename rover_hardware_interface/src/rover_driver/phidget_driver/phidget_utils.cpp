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

#include "rover_hardware_interface/rover_driver/phidget_driver/phidget_utils.hpp"

namespace rover_hardware_interface
{

void openWaitForAttachment(
    PhidgetHandle handle, 
    int32_t serial_number,
    int hub_port, 
    bool is_hub_port_device, 
    int channel)
{
    PhidgetReturnCode ret;

    ret = Phidget_setDeviceSerialNumber(handle, serial_number);
    if (ret != EPHIDGET_OK)
    {
        throw std::runtime_error("Failed to set device serial number hub port " + std::to_string(hub_port) + " channel " + std::to_string(channel));
    }

    ret = Phidget_setHubPort(handle, hub_port);
    if (ret != EPHIDGET_OK)
    {
        throw std::runtime_error("Failed to set device hub port " + std::to_string(hub_port) + " channel " + std::to_string(channel));
    }

    ret = Phidget_setIsHubPortDevice(handle, is_hub_port_device);
    if (ret != EPHIDGET_OK)
    {
        throw std::runtime_error("Failed to set device is hub port " + std::to_string(hub_port) + " channel " + std::to_string(channel));
    }

    ret = Phidget_setChannel(handle, channel);
    if (ret != EPHIDGET_OK)
    {
        throw std::runtime_error("Failed to set device channel hub port " + std::to_string(hub_port) + " channel " + std::to_string(channel));
    }

    ret = Phidget_openWaitForAttachment(handle, PHIDGET_TIMEOUT_DEFAULT);
    if (ret != EPHIDGET_OK)
    {
        throw std::runtime_error("Failed to open device hub port " + std::to_string(hub_port) + " channel " + std::to_string(channel));
    }
}

void closeAndDelete(PhidgetHandle *handle) noexcept
{
    Phidget_close(*handle);
    Phidget_delete(handle);
}

}  // namespace rover_hardware_interface
