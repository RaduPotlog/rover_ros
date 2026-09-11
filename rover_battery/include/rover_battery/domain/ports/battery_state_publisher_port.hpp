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

#ifndef ROVER_BATTERY_DOMAIN_PORTS_BATTERY_STATE_PUBLISHER_PORT_HPP_
#define ROVER_BATTERY_DOMAIN_PORTS_BATTERY_STATE_PUBLISHER_PORT_HPP_

#include "rover_battery/domain/battery_report.hpp"

namespace rover_battery::domain
{

/** @brief Output port: makes a battery report available to the rest of the system. */
class BatteryStatePublisherPort
{
public:
    virtual ~BatteryStatePublisherPort() = default;

    virtual void publish(const BatteryReport & report) = 0;
};

}  // namespace rover_battery::domain

#endif  // ROVER_BATTERY_DOMAIN_PORTS_BATTERY_STATE_PUBLISHER_PORT_HPP_
