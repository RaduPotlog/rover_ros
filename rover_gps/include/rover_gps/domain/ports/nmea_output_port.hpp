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

#ifndef ROVER_GPS_DOMAIN_PORTS_NMEA_OUTPUT_PORT_HPP_
#define ROVER_GPS_DOMAIN_PORTS_NMEA_OUTPUT_PORT_HPP_

#include "rover_gps/domain/nmea/nmea_fix_assembler.hpp"

namespace rover_gps::domain
{

/**
 * @brief Output port for everything the NMEA driver produces.
 * @details `stamp_s` is the receive time in seconds on the node's clock; the adapter converts it
 *          into the message header stamp. It is passed alongside each payload rather than embedded
 *          in it so the assembler stays free of any notion of time-of-receipt.
 */
class NmeaOutputPort
{
public:
    virtual ~NmeaOutputPort() = default;

    virtual void publishFix(const nmea::FixOutput & fix, double stamp_s) = 0;

    virtual void publishVelocity(const nmea::VelocityOutput & velocity, double stamp_s) = 0;

    virtual void publishHeading(const nmea::HeadingOutput & heading, double stamp_s) = 0;

    virtual void publishTimeReference(const nmea::TimeRefOutput & time_ref, double stamp_s) = 0;
};

}  // namespace rover_gps::domain

#endif  // ROVER_GPS_DOMAIN_PORTS_NMEA_OUTPUT_PORT_HPP_
