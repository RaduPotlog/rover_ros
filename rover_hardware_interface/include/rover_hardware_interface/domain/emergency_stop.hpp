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
// hardware. Implemented by an infrastructure adapter (see RoverSafetyControllerEStopIo) so this
// domain class never depends on RoverSafetyController/Modbus directly.
class EmergencyStopIoPort
{

public:

    virtual ~EmergencyStopIoPort() = default;

    virtual bool isUserButtonActive() = 0;

    virtual bool isLatchActive() = 0;

    // The contactor's auxiliary-contact feedback: true while the contacts are closed (motors
    // powered). Genuine plant state, not an echo of anything we commanded - it is what makes the
    // welded-contactor check in ContactorMonitor possible.
    virtual bool isContactorEngaged() = 0;

    virtual void triggerUserButton(const bool state) = 0;

    // SW E-STOP MOTOR DRIVER FAULT - sw_e_stop_motor_driver_fault. The relay's second software SET
    // input; like triggerUserButton(), `true` asserts it.
    virtual void triggerMotorDriverFault(const bool state) = 0;

    virtual void resetLatch() = 0;
};

class EmergencyStopInterface
{

public:

    EmergencyStopInterface() {}

    virtual ~EmergencyStopInterface() = default;

    virtual bool readEStopState() = 0;

    virtual bool readEStopLatchState() = 0;

    virtual bool readContactorEngagedState() = 0;

    virtual void setEStop() = 0;

    virtual void resetEStop() = 0;

    virtual void resetEStopLatch() = 0;

    virtual void releaseStartupTriggers() = 0;
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

    ~EmergencyStop() override = default;

    bool readEStopState() override;

    bool readEStopLatchState() override;

    bool readContactorEngagedState() override;

    void setEStop() override;

    void resetEStop() override;

    void resetEStopLatch() override;

    // Configure-time only: called once from RoverSystem::configureRoverController(), right after
    // the safety controller's start() has asserted both software E-Stop inputs (their initCoils()
    // defaults). Releases the user-button input, then the motor-driver-fault input.
    //
    // Deliberately not resetEStop() and not subject to its zero-velocity invariant: the motor
    // drivers aren't initialised yet (configureRoverDriver() runs after), write() isn't running
    // for this component, and the set-dominant relay latch these inputs SET holds until
    // sw_e_stop_latch_reset. resetEStop() would also refuse on a fresh configure, because the
    // zero-velocity check is fail-safe false until write() has run.
    //
    // Port exceptions propagate unwrapped (unlike setEStop()/resetEStop()), so a failed write
    // reaches on_configure()'s log with the port's own message.
    void releaseStartupTriggers() override;

protected:

    std::shared_ptr<EmergencyStopIoPort> io_;

    std::function<bool()> zero_velocity_check_;

    std::mutex e_stop_manipulation_mtx_;

    // Separate atomics (not one shared flag) so readEStopState() and readEStopLatchState()
    // can't return each other's last-known value under lock contention.
    std::atomic_bool user_e_stop_triggered_ = true;
    std::atomic_bool latch_triggered_ = true;

    // Fail-safe default is `true` (contacts closed / motors possibly live) for the same reason
    // as the two above: before anything has been read, assume the state that demands caution.
    // Paired with a latched E-Stop that is also assumed active, this reads as "disagreement",
    // which ContactorMonitor's drop-out tolerance absorbs long before it could latch a fault.
    std::atomic_bool contactor_engaged_ = true;
};

}  // namespace rover_hardware_interface

#endif  // ROVER_HARDWARE_INTERFACE_DOMAIN_EMERGENCY_STOP_HPP_
