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

#ifndef ROVER_HARDWARE_INTERFACE_DOMAIN_DRIVER_HPP_
#define ROVER_HARDWARE_INTERFACE_DOMAIN_DRIVER_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <future>
#include <memory>
#include <mutex>
#include <string>

namespace rover_hardware_interface
{

enum class MotorNames {
    DEFAULT = 0,
};

enum class DriverNames {
    REAR_LEFT = 0,
    REAR_RIGHT,
    FRONT_LEFT,
    FRONT_RIGHT,
};

inline std::string motorNamesToString(const MotorNames motor_name)
{
    switch (motor_name) {
        case MotorNames::DEFAULT:
            return "default";
        default:
            return "unknown";
    }
}

inline std::string driverNamesToString(const DriverNames driver_name)
{
    switch (driver_name) {
        case DriverNames::REAR_RIGHT:
            return "rear_right";
        case DriverNames::REAR_LEFT:
            return "rear_left";
        case DriverNames::FRONT_RIGHT:
            return "front_right";
        case DriverNames::FRONT_LEFT:
            return "front_left";
        default:
            return "unknown";
    }
}

}  // namespace rover_hardware_interface

namespace std
{

template <>
struct hash<rover_hardware_interface::MotorNames>
{
    std::size_t operator()(const rover_hardware_interface::MotorNames & motor_name) const noexcept
    {
        return static_cast<std::size_t>(motor_name);
    }
};

template <>
struct hash<rover_hardware_interface::DriverNames>
{
    std::size_t operator()(const rover_hardware_interface::DriverNames & driver_name) const noexcept
    {
        return static_cast<std::size_t>(driver_name);
    }
};

}  // namespace std

namespace rover_hardware_interface
{

struct MotorDriverState
{
    std::int64_t pos;
    // Motor-shaft speed in RPM. Kept as double: truncating to an integer RPM throws away the
    // sub-RPM resolution a closed wheel-speed loop needs at low speed.
    double       vel;
    std::int16_t current;
    float        temp;
};

struct DriverState
{
    std::uint8_t fault_flags;
    std::uint8_t runtime_stat_flag;
    std::int16_t driver_current;
    float        temp;
};

class MotorDriverInterface;

class DriverInterface
{

public:

    virtual std::future<void> initialize() = 0;

    virtual DriverState readState() = 0;

    virtual void addMotorDriver(const MotorNames name, std::shared_ptr<MotorDriverInterface> motor_driver) = 0;

    virtual std::shared_ptr<MotorDriverInterface> getMotorDriver(const MotorNames name) = 0;

    virtual bool isCommunicationError() = 0;

    using SharedPtr = std::shared_ptr<DriverInterface>;
};

class MotorDriverInterface
{

public:

    virtual void initialize() = 0;

    virtual MotorDriverState readState() = 0;

    virtual void sendCmdVel(const float cmd) = 0;

    virtual bool isCommunicationError() = 0;

    // Arms (or re-arms) this motor's hardware watchdog with whatever timeout it was configured
    // with. Idempotent - safe to call on an already-armed, non-tripped channel. Not RT-safe (may
    // block on a synchronous SDK/transport call) - only call from on_activate() or a service
    // callback, never from read()/write().
    virtual void armFailsafe() = 0;

    // Explicit, operator-acknowledged clear of a tripped watchdog, re-arming it for further use.
    // Must genuinely recover a tripped channel (e.g. by re-opening it, if the hardware rejects
    // everything until then) - merely feeding the timer is not enough. Kept distinct from armFailsafe() so call sites (RoverSystem::on_activate() vs.
    // RoverSystem::resetEStopLatch()) read as what they mean. Not RT-safe - see armFailsafe().
    virtual void resetFailsafe() = 0;

    // Non-blocking, RT-safe: whether the last command this driver tried to send was rejected
    // because the hardware watchdog had tripped. Latched by the implementation until
    // resetFailsafe() succeeds - never auto-clears on its own.
    virtual bool isFailsafeTripped() = 0;
};

}  // namespace rover_hardware_interface

#endif  // ROVER_HARDWARE_INTERFACE_DOMAIN_DRIVER_HPP_
