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

#ifndef ROVER_LIDAR_DOMAIN_PORTS_LIDAR_HEALTH_PUBLISHER_PORT_HPP_
#define ROVER_LIDAR_DOMAIN_PORTS_LIDAR_HEALTH_PUBLISHER_PORT_HPP_

#include "rover_lidar/domain/lidar_health_evaluator.hpp"

namespace rover_lidar::domain
{

/** @brief Output port: makes the lidar stream health available to the rest of the system. */
class LidarHealthPublisherPort
{
public:
    virtual ~LidarHealthPublisherPort() = default;

    virtual void publish(const LidarHealthReport & report) = 0;
};

}  // namespace rover_lidar::domain

#endif  // ROVER_LIDAR_DOMAIN_PORTS_LIDAR_HEALTH_PUBLISHER_PORT_HPP_
