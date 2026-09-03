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

#ifndef ROVER_HARDWARE_INTERFACE_DOMAIN_EMERGENCY_STOP_HPP_
#define ROVER_HARDWARE_INTERFACE_DOMAIN_EMERGENCY_STOP_HPP_

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>

namespace rover_hardware_interface
{

// Port: the safety-IO primitives EmergencyStop needs from whatever actually talks to the E-Stop
// hardware. Implemented by an infrastructure adapter (see RoverControllerEStopIo) so this domain
// class never depends on RoverController/Modbus directly.
class EmergencyStopIoPort
{

public:

    virtual ~EmergencyStopIoPort() = default;

    virtual bool isUserButtonActive() = 0;

    virtual bool isLatchActive() = 0;

    virtual void triggerUserButton(const bool state) = 0;

    virtual void resetLatch() = 0;
};

class EmergencyStopInterface
{

public:

    EmergencyStopInterface() {}

    virtual ~EmergencyStopInterface() = default;

    virtual bool readEStopState() = 0;

    virtual bool readEStopLatchState() = 0;

    virtual void setEStop() = 0;

    virtual void resetEStop() = 0;

    virtual void resetEStopLatch() = 0;
};

// Owns the "can't clear the E-Stop while the rover is still being commanded to move" safety
// invariant (see resetEStop()). Pure domain logic behind EmergencyStopIoPort so it's unit-testable
// with a fake port, the same way RoverErrorFilter/ImuCalibrationGate are.
class EmergencyStop : public EmergencyStopInterface
{

public:

    EmergencyStop(
        std::shared_ptr<EmergencyStopIoPort> io,
        std::function<bool()> zero_velocity_check);

    virtual ~EmergencyStop() override = default;

    bool readEStopState() override;

    bool readEStopLatchState() override;

    void setEStop() override;

    void resetEStop() override;

    void resetEStopLatch() override;

protected:

    std::shared_ptr<EmergencyStopIoPort> io_;

    std::function<bool()> zeroVelocityCheck;

    std::mutex e_stop_manipulation_mtx_;

    // Separate atomics (not one shared flag) so readEStopState() and readEStopLatchState()
    // can't return each other's last-known value under lock contention.
    std::atomic_bool user_e_stop_triggered_ = true;
    std::atomic_bool latch_triggered_ = true;
};

}  // namespace rover_hardware_interface

#endif  // ROVER_HARDWARE_INTERFACE_DOMAIN_EMERGENCY_STOP_HPP_
