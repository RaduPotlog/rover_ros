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

#ifndef ROVER_HARDWARE_INTERFACE_ROVER_DRIVER_PHIDGET_MOTOR_DRIVER_PHIDGET_MOTOR_DRIVER_HPP_
#define ROVER_HARDWARE_INTERFACE_ROVER_DRIVER_PHIDGET_MOTOR_DRIVER_PHIDGET_MOTOR_DRIVER_HPP_

#include <libphidget22/phidget22.h>

#include <functional>
#include <vector>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>

#include "rover_hardware_interface/domain/driver.hpp"
#include "rover_hardware_interface/utils.hpp"

namespace rover_hardware_interface
{

class PhidgetDriver : public DriverInterface
{

public:

    PhidgetDriver();

    std::future<void> initialize() override;

    DriverState readState() override;

    void addMotorDriver(const MotorNames name, std::shared_ptr<MotorDriverInterface> motor_driver) override;

    std::shared_ptr<MotorDriverInterface> getMotorDriver(const MotorNames name) override;

    bool isCommunicationError() override;

private:

    std::mutex init_mtx_;
    std::promise<void> init_promise_;

    std::unordered_map<MotorNames, std::shared_ptr<MotorDriverInterface>> motor_drivers_;

    rclcpp::Logger logger_{rclcpp::get_logger("PhidgetDriver")};
};

class PhidgetMotorDriver : public MotorDriverInterface
{

public:

    PhidgetMotorDriver(
        const DrivetrainSettings & drivetrain_settings,
        std::weak_ptr<PhidgetDriver> driver,
        const std::uint8_t channel,
        const std::int32_t serial_number,
        const bool dir_reverse);

    ~PhidgetMotorDriver();

    void initialize() override;

    MotorDriverState readState() override;

    void sendCmdVel(const float cmd) override;

    bool isCommunicationError() override;

    void armFailsafe() override;

    void resetFailsafe() override;

    bool isFailsafeTripped() override;

    // Pure logic, factored out so it's unit-testable without any Phidget SDK handles: whether a
    // PhidgetReturnCode reported by the async setTargetVelocity completion means the hardware
    // watchdog rejected the command.
    static bool isFailsafeTrippedReturnCode(const PhidgetReturnCode res);

    // What armFailsafe() has to do to leave the channel with a working, freshly-fed watchdog.
    // A Phidget channel's failsafe can only be enabled once per open, and once it has tripped the
    // channel rejects every further call until it is closed and re-opened (Phidget Failsafe
    // Guide) - so neither PhidgetDCMotor_resetFailsafe() nor PhidgetDCMotor_enableFailsafe() can
    // recover a tripped channel on their own.
    enum class FailsafeAction
    {
        kEnable,           // Never enabled on this open channel - enable it.
        kFeed,             // Already enabled and healthy - just reset (feed) the timer.
        kReopenAndEnable,  // Tripped - close/re-open the channel, reconfigure, then enable.
    };

    // Pure logic, factored out so it's unit-testable without any Phidget SDK handles.
    static FailsafeAction selectFailsafeAction(const bool enabled, const bool tripped);

    // Motor-shaft RPM from a change of raw quadrature counts over dt_s seconds, for an encoder
    // with `lines` lines per revolution (4 counts per line). Works on raw counts so no remainder
    // is dropped between events. Returns 0 for a non-positive dt_s or lines.
    static double encoderCountsToMotorRpm(
        const std::int64_t delta_counts, const double dt_s, const float lines);

    // How long without an encoder event before the reported speed is forced to 0, for an encoder
    // reporting every `data_interval_ms`. Without it a wheel that stops keeps reporting its last
    // non-zero speed if the callback doesn't fire at standstill. 3x the interval so one or two
    // late/missed events don't zero the speed of a turning wheel - the DCC1000 encoder can't
    // report faster than every 50 ms, so the timeout must never be near a single interval.
    static std::chrono::nanoseconds encoderStaleTimeout(const std::uint32_t data_interval_ms);

private:

    // Applies the motor channel's settings (acceleration, current limit, regulator gain, braking)
    // to the open motor_handle_. Used by initialize() and again after reopenMotorChannel(), since
    // closing the channel drops them.
    void configureMotorChannel();

    // Closes and re-opens motor_handle_ (the same handle - never deleted, so a concurrent RT
    // sendCmdVel() on it just fails, it never dangles) and re-applies configureMotorChannel().
    // The only way out of a tripped failsafe state. Blocking - never call from the RT path.
    void reopenMotorChannel();

    // PhidgetDCMotor_enableFailsafe() with the configured timeout; throws on failure.
    void enableFailsafe();

    // "0x3b (Failsafe Triggered)"-style text for a PhidgetReturnCode, for exception messages.
    static std::string returnCodeToString(const PhidgetReturnCode ret);

    // Pure logic, factored out so it's unit-testable without any Phidget SDK handles.
    static bool isCommTimedOut(
        const std::chrono::steady_clock::time_point & last_update,
        const std::chrono::steady_clock::time_point & now,
        const std::chrono::nanoseconds & timeout);

    static void CCONV positionChangeHandler(
        PhidgetEncoderHandle phid,
        void * ctx,
        int positionChange,
        double timeChange,
        int indexTriggered);

    static void CCONV currentChangeHandler(
        PhidgetCurrentInputHandle phid,
        void * ctx,
        double current
    );

    static void CCONV temperatureChangeHandler(
        PhidgetTemperatureSensorHandle phid,
        void *ctx,
        double temperature
    );

    static void CCONV setTargetVelocityHandler(
        PhidgetHandle phid,
        void * ctx, PhidgetReturnCode res);

    std::weak_ptr<PhidgetDriver> driver_;

    const std::uint8_t channel_;
    std::int32_t serial_number_;

    PhidgetDCMotorHandle motor_handle_{nullptr};
    PhidgetEncoderHandle encoder_handle_{nullptr};
    PhidgetCurrentInputHandle current_handle_{nullptr};
    PhidgetTemperatureSensorHandle temperature_handle_{nullptr};

    int64_t encoder_ticks_{0};
    int64_t prev_encoder_ticks_{0};

    double position_time_change_{0.0f};

    // `state_` is written from the Phidget SDK's callback thread
    // (positionChangeHandler/currentChangeHandler/temperatureChangeHandler) and read from the
    // RT control thread via readState(). `state_mtx_` guards `state_` itself; `state_snapshot_`
    // is the last successfully-read copy, only ever touched from readState() on the RT thread,
    // so readState() never blocks the RT loop waiting for the SDK thread to release the lock.
    std::mutex state_mtx_;
    MotorDriverState state_;
    MotorDriverState state_snapshot_{};

    float encoder_resolution_;

    float motor_acceleration_;

    // steady_clock ns of the last encoder event; written on the SDK thread, read in readState().
    std::atomic<std::int64_t> last_encoder_event_ns_{0};

    // encoderStaleTimeout() of the interval the encoder actually runs at; set in initialize().
    std::atomic<std::int64_t> encoder_stale_timeout_ns_{
        encoderStaleTimeout(50).count()};

    bool direction_reversed_;

    // true while a PhidgetDCMotor_setTargetVelocity_async() call is in flight (cleared by
    // setTargetVelocityHandler() once the SDK reports completion).
    std::atomic<bool> set_speed_pending_{false};

    // Set by setTargetVelocityHandler() (Phidget SDK callback thread) when a command completion
    // reports the watchdog has tripped; read via isFailsafeTripped() (RT thread). Only cleared by
    // resetFailsafe() - an operator-driven action - so a trip stays latched here even if later
    // commands' completions report success again (they won't, until resetFailsafe() re-opens the
    // channel, but the latch is intentional defense-in-depth regardless).
    std::atomic<bool> failsafe_tripped_{false};

    // Whether PhidgetDCMotor_enableFailsafe() has succeeded on the currently open motor channel.
    // Cleared by reopenMotorChannel() (closing the channel disables the failsafe). Only touched
    // from armFailsafe()/resetFailsafe(), serialized by failsafe_mtx_.
    bool failsafe_enabled_{false};

    // Serializes armFailsafe()/resetFailsafe() (on_activate() vs. the latch-reset service).
    std::mutex failsafe_mtx_;

    // Written from the Phidget SDK callback thread (position/current/temperature handlers) each
    // time a telemetry callback fires — the firing itself is the liveness signal, independent of
    // the value carried. Read from the RT thread via isCommunicationError(). A plain
    // atomic<int64_t> nanosecond count (steady_clock epoch) is used instead of state_mtx_: it's
    // an independent scalar, not part of the MotorDriverState struct that mutex guards, so a
    // lock-free atomic avoids adding contention to the already try_lock'd state_mtx_ path.
    std::atomic<std::int64_t> last_update_time_ns_{0};

    const std::chrono::nanoseconds comm_timeout_;

    // Timeout armFailsafe() arms PhidgetDCMotor_enableFailsafe() with. Kept as a member (rather
    // than a call-site literal) so resetFailsafe()'s re-open-and-re-arm recovery (see
    // phidget_motor_driver.cpp) uses the same configured value.
    const std::uint32_t failsafe_timeout_ms_;

    rclcpp::Logger logger_{rclcpp::get_logger("PhidgetMotorDriver")};
};

}  // namespace rover_hardware_interface

#endif  // ROVER_HARDWARE_INTERFACE_ROVER_DRIVER_PHIDGET_MOTOR_DRIVER_PHIDGET_MOTOR_DRIVER_HPP_
