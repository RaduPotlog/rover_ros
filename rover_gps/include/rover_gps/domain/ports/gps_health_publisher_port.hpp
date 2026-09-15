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

#ifndef ROVER_GPS_DOMAIN_PORTS_GPS_HEALTH_PUBLISHER_PORT_HPP_
#define ROVER_GPS_DOMAIN_PORTS_GPS_HEALTH_PUBLISHER_PORT_HPP_

#include "rover_gps/domain/gps_health_evaluator.hpp"

namespace rover_gps::domain
{

/** @brief Output port: makes the GNSS stream health available to the rest of the system. */
class GpsHealthPublisherPort
{
public:
    virtual ~GpsHealthPublisherPort() = default;

    virtual void publish(const GpsHealthReport & report) = 0;
};

}  // namespace rover_gps::domain

#endif  // ROVER_GPS_DOMAIN_PORTS_GPS_HEALTH_PUBLISHER_PORT_HPP_
