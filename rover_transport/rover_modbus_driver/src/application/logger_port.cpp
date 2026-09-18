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

#include "rover_modbus_driver/domain/logger_port.hpp"

#include <iostream>

namespace rover::transport::modbus
{

void StdErrLogger::warn(const std::string & message)
{
    std::cerr << "[WARN] [RoverModbus]: " << message << std::endl;
}

void StdErrLogger::error(const std::string & message)
{
    std::cerr << "[ERROR] [RoverModbus]: " << message << std::endl;
}

}  // namespace rover::transport::modbus
