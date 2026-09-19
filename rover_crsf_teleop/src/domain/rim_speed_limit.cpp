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

#include "rover_crsf_teleop/domain/rim_speed_limit.hpp"

#include <cmath>

namespace rover_crsf_teleop
{

VelocityCommand limitRimSpeed(
    const VelocityCommand & command, const double max_rim_speed, const double half_track)
{
    if (max_rim_speed <= 0.0 || half_track <= 0.0) {
        return command;
    }

    const double rim_speed = std::abs(command.linear_x) + std::abs(command.angular_z) * half_track;
    if (rim_speed <= max_rim_speed) {
        return command;
    }

    const double scale = max_rim_speed / rim_speed;
    return VelocityCommand{command.linear_x * scale, command.angular_z * scale};
}

}  // namespace rover_crsf_teleop
