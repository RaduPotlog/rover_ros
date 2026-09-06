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

#include "rover_hardware_interface/domain/velocity_command_guard.hpp"

#include <cmath>
#include <vector>

namespace rover_hardware_interface
{

bool areVelocitiesWithinTolerance(const std::vector<double> & commands, const double tolerance)
{
    for (const auto & command : commands) {
        if (!std::isfinite(command) || std::abs(command) > tolerance) {
            return false;
        }
    }

    return true;
}

}  // namespace rover_hardware_interface
