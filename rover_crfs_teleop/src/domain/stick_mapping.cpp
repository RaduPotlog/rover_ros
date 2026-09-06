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

#include "rover_crfs_teleop/domain/stick_mapping.hpp"

#include <algorithm>
#include <cstdlib>

namespace rover_crfs_telop
{

double mapAxis(const int raw_value, const AxisMapping & mapping)
{
    const int clamped = std::min(std::max(raw_value, mapping.in_min), mapping.in_max);
    const int deflection = clamped - mapping.in_mid;

    if (std::abs(deflection) <= mapping.deadband_counts) {
        return 0.0;
    }

    // The two halves of the throw are normalized independently, so an asymmetric channel range
    // (in_mid not exactly midway between in_min and in_max, which is the norm once a transmitter
    // has been trimmed) still reaches both output limits without shifting the centre off zero.
    const int half_span = (deflection > 0) ? (mapping.in_max - mapping.in_mid - mapping.deadband_counts)
                                           : (mapping.in_mid - mapping.in_min - mapping.deadband_counts);

    if (half_span <= 0) {
        return 0.0;
    }

    const double magnitude = static_cast<double>(std::abs(deflection) - mapping.deadband_counts) /
                             static_cast<double>(half_span);

    double normalized = (deflection > 0) ? magnitude : -magnitude;

    if (mapping.invert) {
        normalized = -normalized;
    }

    return (normalized >= 0.0) ? (normalized * mapping.out_max) : (-normalized * mapping.out_min);
}

}  // namespace rover_crfs_telop
