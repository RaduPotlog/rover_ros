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

#ifndef ROVER_RS16_LIDAR_DOMAIN_PORTS_LASER_SCAN_PUBLISHER_PORT_HPP_
#define ROVER_RS16_LIDAR_DOMAIN_PORTS_LASER_SCAN_PUBLISHER_PORT_HPP_

#include "rover_rs16_lidar/domain/laser_scan_frame.hpp"

namespace rover_rs16_lidar::domain
{

/** @brief Output port: makes the flattened scan available to the rest of the system. */
class LaserScanPublisherPort
{
public:
    virtual ~LaserScanPublisherPort() = default;

    virtual void publish(const LaserScanFrame & scan) = 0;
};

}  // namespace rover_rs16_lidar::domain

#endif  // ROVER_RS16_LIDAR_DOMAIN_PORTS_LASER_SCAN_PUBLISHER_PORT_HPP_
