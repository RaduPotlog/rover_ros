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


#include "rover_crfs_teleop/domain/teleop_health.hpp"

#include <string>

namespace rover_crfs_teleop
{

HealthReport evaluateTeleopHealth(const bool first_frame_received, const LinkHealthSnapshot & link)
{
    if (!first_frame_received) {
        return {HealthLevel::kWarn, "Waiting for the first rc/channels frame."};
    }

    if (link.loss_reason != LinkLossReason::kNone) {
        return {HealthLevel::kError, std::string("RC link lost: ") + toString(link.loss_reason) + "."};
    }

    return {HealthLevel::kOk, "RC link healthy."};
}

}  // namespace rover_crfs_teleop
