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

#ifndef ROVER_CRSF_TELEOP_DOMAIN_RIM_SPEED_LIMIT_HPP_
#define ROVER_CRSF_TELEOP_DOMAIN_RIM_SPEED_LIMIT_HPP_

#include "rover_crsf_teleop/domain/ports.hpp"

namespace rover_crsf_teleop
{

// Keeps the outer wheel of a skid-steer base within a rim-speed budget.
//
// diff_drive limits linear.x and angular.z independently, so full forward plus full turn asks the
// outer wheel for |v| + |w| * half_track, which can far exceed what the wheel joints allow. When the
// joint limiter then clips only the outer wheel, the rover turns tighter than the sticks asked for.
// Scaling v and w by the same factor instead keeps the curvature w/v, so the arc is the one
// commanded, just driven slower.
//
// Guarantees:
//   - a command within the budget is returned unchanged;
//   - otherwise v and w are scaled by one common factor so the rim speed equals the budget;
//   - a zero command stays exactly zero (the hardware E-Stop reset needs a true zero);
//   - max_rim_speed <= 0 or half_track <= 0 disables the limit (input returned unchanged).
VelocityCommand limitRimSpeed(
    const VelocityCommand & command, double max_rim_speed, double half_track);

}  // namespace rover_crsf_teleop

#endif  // ROVER_CRSF_TELEOP_DOMAIN_RIM_SPEED_LIMIT_HPP_
