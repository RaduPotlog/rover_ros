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


#ifndef ROVER_CRFS_TELEOP_DOMAIN_TELEOP_HEALTH_HPP_
#define ROVER_CRFS_TELEOP_DOMAIN_TELEOP_HEALTH_HPP_

#include <string>

#include "rover_crfs_teleop/domain/link_monitor.hpp"

namespace rover_crfs_teleop
{

// Ordered by severity. There is deliberately no error level: RC teleop is optional, so its
// diagnostics warn at worst.
enum class HealthLevel
{
    kOk,
    kWarn,
};

struct HealthReport
{
    HealthLevel level{HealthLevel::kOk};
    std::string message;
};

// Grades the RC link: WARN until the first rc/channels frame, ERROR with the reason while the
// link is lost, OK otherwise. Uses the same LinkMonitor verdict that gates the velocity command,
// so the diagnostic cannot disagree with what teleop actually does.
HealthReport evaluateTeleopHealth(bool first_frame_received, const LinkHealthSnapshot & link);

}  // namespace rover_crfs_teleop

#endif  // ROVER_CRFS_TELEOP_DOMAIN_TELEOP_HEALTH_HPP_
