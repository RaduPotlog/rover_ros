// Copyright 2026 Mechatronics Academy
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

#ifndef ROVER_MODBUS_DRIVER_DOMAIN_CLIENT_SETTINGS_HPP_
#define ROVER_MODBUS_DRIVER_DOMAIN_CLIENT_SETTINGS_HPP_

#include <string>

namespace rover::transport::modbus
{

// Default response timeout, matching MB::TCP::Connection::DefaultTCPTimeout. Named here
// so a caller that leaves response_timeout_ms unset gets the documented value rather
// than whatever the transport library happens to default to.
constexpr unsigned kDefaultResponseTimeoutMs = 500;

// Connection settings for a Modbus TCP client. This was ModbusSettings in
// rover_hardware_interface's modbus_types.hpp; it moved with the client because it is
// exactly that client's constructor-argument bundle.
//
// It stays transport configuration, owned by whichever caller builds the client - it is
// not domain state of the robot. In the rover's case the values are parsed out of the
// URDF <ros2_control> hardware parameters by RoverA1System, which is a ros2_control
// plugin concern and deliberately stays there.
struct ClientSettings
{
    std::string host;
    int port;
    unsigned connection_retry_count;      // 0 = retry forever
    unsigned connection_retry_delay_ms;
    unsigned response_timeout_ms = kDefaultResponseTimeoutMs;
};

}  // namespace rover::transport::modbus

#endif  // ROVER_MODBUS_DRIVER_DOMAIN_CLIENT_SETTINGS_HPP_
