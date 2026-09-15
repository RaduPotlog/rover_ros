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

#ifndef ROVER_GPS_DOMAIN_PORTS_HEADING_PUBLISHER_PORT_HPP_
#define ROVER_GPS_DOMAIN_PORTS_HEADING_PUBLISHER_PORT_HPP_

#include "rover_gps/domain/heading_alignment_estimator.hpp"

namespace rover_gps::domain
{

/** @brief Absolute (ENU) heading of the base, derived from odometry and the GNSS alignment. */
struct EnuHeading
{
    double stamp_s{0.0};
    double yaw_rad{0.0};      // 0 = east, counter-clockwise positive
    double yaw_std_rad{0.0};
};

/** @brief Output port for the heading and the alignment progress. */
class HeadingPublisherPort
{
public:
    virtual ~HeadingPublisherPort() = default;

    virtual void publishHeading(const EnuHeading & heading) = 0;

    virtual void publishAlignmentStatus(const AlignmentStatus & status) = 0;
};

}  // namespace rover_gps::domain

#endif  // ROVER_GPS_DOMAIN_PORTS_HEADING_PUBLISHER_PORT_HPP_
