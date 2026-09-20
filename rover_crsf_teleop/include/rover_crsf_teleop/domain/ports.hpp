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

#ifndef ROVER_CRSF_TELEOP_DOMAIN_PORTS_HPP_
#define ROVER_CRSF_TELEOP_DOMAIN_PORTS_HPP_

#include <optional>
#include <string>

#include "rover_crsf_teleop/domain/rc_calibration.hpp"

namespace rover_crsf_teleop
{

// Planar velocity command in the robot frame: m/s and rad/s.
struct VelocityCommand
{
    double linear_x{0.0};
    double angular_z{0.0};

    bool isZero() const { return linear_x == 0.0 && angular_z == 0.0; }
};

// Where teleop velocity commands go. Implemented in infrastructure by a TwistStamped publisher
// feeding twist_mux.
class VelocityCommandPort
{

public:

    virtual ~VelocityCommandPort() = default;

    virtual void publish(const VelocityCommand & command) = 0;
};

// The rover's software E-Stop, as driven from the RC switches. Implemented in infrastructure by
// std_srvs/Trigger clients on the hardware interface. Requests are fire-and-forget: the hardware
// interface is the authority on whether one is accepted.
class SafetySwitchPort
{

public:

    virtual ~SafetySwitchPort() = default;

    virtual void requestUserEStopSet() = 0;

    virtual void requestUserEStopReset() = 0;

    virtual void requestLatchReset() = 0;
};

// A measured calibration plus where it came from. The timestamp is a preformatted ISO-8601 string
// so the domain still needs no clock; the infrastructure adapter formats it.
struct StoredCalibration
{
    int schema_version{1};
    std::string created;
    ChannelCalibration calibration;
};

// Where a measured calibration is kept between runs. Implemented in infrastructure by a yaml-cpp
// file store on a persistent volume; a null store means persistence is turned off.
//
// Never throws: an unreadable or malformed store reports nullopt / false after the adapter has
// logged why. A calibration that cannot be saved is still worth applying live, so the caller
// treats a failed save as a warning, not as a failed apply.
class CalibrationStorePort
{

public:

    virtual ~CalibrationStorePort() = default;

    virtual std::optional<StoredCalibration> load() = 0;

    virtual bool save(const ChannelCalibration & calibration, std::string & error) = 0;

    // Where this store reads and writes, for log lines and service responses.
    virtual std::string location() const = 0;
};

}  // namespace rover_crsf_teleop

#endif  // ROVER_CRSF_TELEOP_DOMAIN_PORTS_HPP_
