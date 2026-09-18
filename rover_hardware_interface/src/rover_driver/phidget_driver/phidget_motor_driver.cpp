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

#include "rover_hardware_interface/rover_driver/phidget_driver/phidget_motor_driver.hpp"

#include <cstdio>
#include <string>

#include "rover_hardware_interface/domain/driver.hpp"
#include "rover_hardware_interface/rover_driver/phidget_driver/phidget_utils.hpp"

namespace rover_hardware_interface
{

PhidgetDriver::PhidgetDriver()
{

}

std::future<void> PhidgetDriver::initialize()
{
    std::lock_guard<std::mutex> lck(init_mtx_);
    init_promise_ = std::promise<void>();

    std::future<void> future = init_promise_.get_future();

    for (auto & [name, motor_driver] : motor_drivers_) {
        try {
            motor_driver->initialize();
        } catch (const std::runtime_error & e) {
            throw std::runtime_error(
                "Motor driver initilize exception on " +
                motorNamesToString(name) +
                " motor: " +
                std::string(e.what()));
        }
    }

    try {
        init_promise_.set_value();
    } catch (const std::future_error & e) {
        if (e.code() == std::make_error_code(std::future_errc::promise_already_satisfied)) {
            RCLCPP_WARN_STREAM(logger_, "An exception occurred while setting init promise: " << e.what());
        }
    }

    return future;
}

DriverState PhidgetDriver::readState()
{
    DriverState driver_state;

    const auto motor_state = this->getMotorDriver(MotorNames::DEFAULT)->readState();
    driver_state.fault_flags = 0;
    driver_state.runtime_stat_flag = 0;
    driver_state.driver_current = motor_state.current;
    driver_state.temp = motor_state.temp;

    return driver_state;
}

bool PhidgetDriver::isCommunicationError()
{
    return getMotorDriver(MotorNames::DEFAULT)->isCommunicationError();
}

void PhidgetDriver::addMotorDriver(
    const MotorNames name,
    std::shared_ptr<MotorDriverInterface> motor_driver)
{
    if (std::dynamic_pointer_cast<PhidgetMotorDriver>(motor_driver) == nullptr) {
        throw std::runtime_error("Motor driver is not of type PhidgetMotorDriver");
    }

    motor_drivers_.emplace(name, motor_driver);
}

std::shared_ptr<MotorDriverInterface> PhidgetDriver::getMotorDriver(const MotorNames name)
{
    auto it = motor_drivers_.find(name);

    if (it == motor_drivers_.end()) {
        throw std::runtime_error("Motor driver with name '" +
            motorNamesToString(name) +
            "' does not exist");
    }

    return it->second;
}

PhidgetMotorDriver::PhidgetMotorDriver(
    const DrivetrainSettings & drivetrain_settings,
    std::weak_ptr<PhidgetDriver> driver,
    const std::uint8_t channel,
    const std::int32_t serial_number,
    const bool dir_reverse)
: driver_(driver)
, channel_(channel)
, serial_number_(serial_number)
, direction_reversed_(dir_reverse)
, comm_timeout_(std::chrono::milliseconds(drivetrain_settings.driver_comm_timeout_ms))
, failsafe_timeout_ms_(drivetrain_settings.motor_failsafe_timeout_ms)
{
    encoder_resolution_ = drivetrain_settings.encoder_resolution;
    motor_acceleration_ = drivetrain_settings.motor_acceleration;

    RCLCPP_INFO(logger_, "Create phidget motor driver channel = %d, encoder resolution = %f", channel_, encoder_resolution_);
}

PhidgetMotorDriver::~PhidgetMotorDriver()
{
    RCLCPP_INFO(logger_, "Destroy phidget motor driver channel = %d", channel_);

    PhidgetHandle handle = reinterpret_cast<PhidgetHandle>(motor_handle_);
    closeAndDelete(&handle);

    handle = reinterpret_cast<PhidgetHandle>(encoder_handle_);
    closeAndDelete(&handle);

    handle = reinterpret_cast<PhidgetHandle>(current_handle_);
    closeAndDelete(&handle);

    handle = reinterpret_cast<PhidgetHandle>(temperature_handle_);
    closeAndDelete(&handle);
}

void PhidgetMotorDriver::initialize()
{
    PhidgetReturnCode ret = PhidgetDCMotor_create(&motor_handle_);

    if (ret != EPHIDGET_OK) {
        throw std::runtime_error("Failed to create Motor handle for channel " +
            std::to_string(channel_));
    }

    openWaitForAttachment(reinterpret_cast<PhidgetHandle>(motor_handle_), -1, channel_, false, 0);

    if (serial_number_ == -1)
    {
        ret = Phidget_getDeviceSerialNumber(reinterpret_cast<PhidgetHandle>(motor_handle_), &serial_number_);

        if (ret != EPHIDGET_OK) {
            throw std::runtime_error("Failed to get serial number for motor channel " +
                std::to_string(channel_));
        }
    }

    // Try to reset fail safe
    (void)PhidgetDCMotor_resetFailsafe(motor_handle_);

    configureMotorChannel();

    int enabled = 0;

    ret = PhidgetEncoder_create(&encoder_handle_);

    if (ret != EPHIDGET_OK) {
        throw std::runtime_error("Failed to create Motor encoder handle for channel " +
            std::to_string(channel_));
    }

    openWaitForAttachment(reinterpret_cast<PhidgetHandle>(encoder_handle_), -1, channel_, false, 0);

    ret = PhidgetEncoder_getEnabled(encoder_handle_, &enabled);

    if (ret != EPHIDGET_OK) {
        throw std::runtime_error("Failed to get enable of the encoder for motor channel " +
            std::to_string(channel_));
    }

    if (enabled == 0) {
        throw std::runtime_error("Encoder not enabled for motor channel " +
            std::to_string(channel_));
    }

    ret = PhidgetEncoder_setIOMode(encoder_handle_, ENCODER_IO_MODE_OPEN_COLLECTOR_10K);

    if (ret != EPHIDGET_OK) {
        throw std::runtime_error("Failed to set mode of the encoder for motor channel " +
            std::to_string(channel_));
    }

    double minDataRate = 0.0;

    ret = PhidgetEncoder_getMinDataRate(encoder_handle_, &minDataRate);

    if (ret != EPHIDGET_OK) {
        throw std::runtime_error("Failed to get minimum data rate of the encoder for motor channel " +
            std::to_string(channel_));
    }

    ret = PhidgetEncoder_setDataRate(encoder_handle_, minDataRate);

    if (ret != EPHIDGET_OK) {
        throw std::runtime_error("Failed to set minimum data rate of the encoder for motor channel " +
            std::to_string(channel_));
    }

    uint32_t minDataInterval = 0;

    ret = PhidgetEncoder_getMinDataInterval(encoder_handle_, &minDataInterval);

    if (ret != EPHIDGET_OK) {
        throw std::runtime_error("Failed to get minimum data interval of the encoder for motor channel " +
            std::to_string(channel_));
    }

    ret = PhidgetEncoder_setDataInterval(encoder_handle_, minDataInterval);

    if (ret != EPHIDGET_OK) {
        throw std::runtime_error("Failed to set minimum data interval of the encoder for motor channel " +
            std::to_string(channel_));
    }

    ret = PhidgetEncoder_setOnPositionChangeHandler(encoder_handle_, positionChangeHandler, this);

    if (ret != EPHIDGET_OK) {
        throw std::runtime_error("Failed to set position change handler of the encoder for motor channel " +
            std::to_string(channel_));
    }

    ret = PhidgetCurrentInput_create(&current_handle_);

    if (ret != EPHIDGET_OK) {
         throw std::runtime_error("Failed to create Current input handle for channel " +
            std::to_string(channel_));
    }

    ret = PhidgetCurrentInput_setOnCurrentChangeHandler(current_handle_, currentChangeHandler, this);

    if (ret != EPHIDGET_OK) {
        throw std::runtime_error("Failed to set current callback for channel " +
            std::to_string(channel_));
    }

    openWaitForAttachment(reinterpret_cast<PhidgetHandle>(current_handle_), -1, channel_, false, 0);

    // Force the current channel to report on every data interval rather than only when the
    // reading changes — this callback firing is also used as a communication-liveness signal
    // (see last_update_time_ns_), and a stationary/steady-current robot must not look "stale".
    uint32_t current_min_interval = 0;

    ret = PhidgetCurrentInput_getMinDataInterval(current_handle_, &current_min_interval);

    if (ret != EPHIDGET_OK) {
        throw std::runtime_error("Failed to get minimum data interval of the current input for motor channel " +
            std::to_string(channel_));
    }

    ret = PhidgetCurrentInput_setDataInterval(current_handle_, current_min_interval);

    if (ret != EPHIDGET_OK) {
        throw std::runtime_error("Failed to set minimum data interval of the current input for motor channel " +
            std::to_string(channel_));
    }

    ret = PhidgetCurrentInput_setCurrentChangeTrigger(current_handle_, 0.0);

    if (ret != EPHIDGET_OK) {
        throw std::runtime_error("Failed to set zero change trigger of the current input for motor channel " +
            std::to_string(channel_));
    }

    ret = PhidgetTemperatureSensor_create(&temperature_handle_);

    if (ret != EPHIDGET_OK) {
        throw std::runtime_error("Failed to create Temperature handle for channel " +
            std::to_string(channel_));
    }

    ret = PhidgetTemperatureSensor_setOnTemperatureChangeHandler(temperature_handle_, temperatureChangeHandler, this);

    if (ret != EPHIDGET_OK) {
        throw std::runtime_error("Failed to set temperature callback for channel " +
            std::to_string(channel_));
    }

    openWaitForAttachment(reinterpret_cast<PhidgetHandle>(temperature_handle_), -1, channel_, false, 0);

    // Same rationale as the current channel above: force periodic reporting so it can serve as
    // a communication-liveness signal even when the temperature reading is not changing.
    uint32_t temperature_min_interval = 0;

    ret = PhidgetTemperatureSensor_getMinDataInterval(temperature_handle_, &temperature_min_interval);

    if (ret != EPHIDGET_OK) {
        throw std::runtime_error("Failed to get minimum data interval of the temperature sensor for motor channel " +
            std::to_string(channel_));
    }

    ret = PhidgetTemperatureSensor_setDataInterval(temperature_handle_, temperature_min_interval);

    if (ret != EPHIDGET_OK) {
        throw std::runtime_error("Failed to set minimum data interval of the temperature sensor for motor channel " +
            std::to_string(channel_));
    }

    ret = PhidgetTemperatureSensor_setTemperatureChangeTrigger(temperature_handle_, 0.0);

    if (ret != EPHIDGET_OK) {
        throw std::runtime_error("Failed to set zero change trigger of the temperature sensor for motor channel " +
            std::to_string(channel_));
    }
}

void PhidgetMotorDriver::configureMotorChannel()
{
    // Set acceleration
    PhidgetReturnCode ret = PhidgetDCMotor_setAcceleration(motor_handle_, motor_acceleration_);

    if (ret != EPHIDGET_OK) {
        throw std::runtime_error("Failed to set acceleration for motor channel " +
            std::to_string(channel_));
    }

    // Set current limit
    ret = PhidgetDCMotor_setCurrentLimit(motor_handle_, 10.0);

    if (ret != EPHIDGET_OK) {
        throw std::runtime_error("Failed to set current limit for motor channel " +
            std::to_string(channel_));
    }

    /*
     *  CurrentRegulatorGain = CurrentLimit * (Voltage / 12)
     */
    ret = PhidgetDCMotor_setCurrentRegulatorGain(motor_handle_, 20.0);

    if (ret != EPHIDGET_OK) {
        throw std::runtime_error("Failed to set current regulator gain for motor channel " +
            std::to_string(channel_));
    }

    ret = PhidgetDCMotor_setTargetBrakingStrength(motor_handle_, 1.0f);

    if (ret != EPHIDGET_OK) {
        throw std::runtime_error("Failed to set braking strength for motor channel " +
            std::to_string(channel_));
    }
}

void PhidgetMotorDriver::reopenMotorChannel()
{
    const PhidgetHandle handle = reinterpret_cast<PhidgetHandle>(motor_handle_);

    // Closing also disables the failsafe - it can't be turned off any other way.
    (void)Phidget_close(handle);
    failsafe_enabled_ = false;

    // Same addressing as initialize() (hub port = channel_), plus the serial number initialize()
    // already resolved, so we re-attach to the very same board.
    openWaitForAttachment(handle, serial_number_, channel_, false, 0);

    configureMotorChannel();

    // Any async command in flight on the old channel has been aborted by the close; don't let a
    // lost completion callback gate sendCmdVel() forever.
    set_speed_pending_ = false;
}

void PhidgetMotorDriver::enableFailsafe()
{
    const PhidgetReturnCode ret =
        PhidgetDCMotor_enableFailsafe(motor_handle_, failsafe_timeout_ms_);

    if (ret != EPHIDGET_OK) {
        throw std::runtime_error(
            "Failed to arm fail safe (timeout " + std::to_string(failsafe_timeout_ms_) +
            "ms) for motor channel " + std::to_string(channel_) + ": " + returnCodeToString(ret));
    }

    failsafe_enabled_ = true;
}

std::string PhidgetMotorDriver::returnCodeToString(const PhidgetReturnCode ret)
{
    char hex[16];
    std::snprintf(hex, sizeof(hex), "0x%02x", static_cast<unsigned>(ret));

    const char * description = nullptr;

    if (Phidget_getErrorDescription(ret, &description) == EPHIDGET_OK && description != nullptr) {
        return std::string(hex) + " (" + description + ")";
    }

    return std::string(hex);
}

MotorDriverState PhidgetMotorDriver::readState()
{
    // Non-blocking: never stall the RT control loop waiting on the SDK callback thread.
    // On contention, just return the last successfully-read snapshot.
    std::unique_lock<std::mutex> lck(state_mtx_, std::try_to_lock);

    if (lck.owns_lock()) {
        state_snapshot_ = state_;
    }

    const auto last_encoder_event_ns = last_encoder_event_ns_.load(std::memory_order_relaxed);

    if (last_encoder_event_ns == 0 ||
        isCommTimedOut(
            std::chrono::steady_clock::time_point(std::chrono::nanoseconds(last_encoder_event_ns)),
            std::chrono::steady_clock::now(), kEncoderStaleTimeout))
    {
        state_snapshot_.vel = 0.0;
    }

    return state_snapshot_;
}

double PhidgetMotorDriver::encoderCountsToMotorRpm(
    const std::int64_t delta_counts, const double dt_s, const float lines)
{
    if (dt_s <= 0.0 || lines <= 0.0f) {
        return 0.0;
    }

    const double revolutions = static_cast<double>(delta_counts) / (4.0 * static_cast<double>(lines));

    return revolutions * 60.0 / dt_s;
}

bool PhidgetMotorDriver::isCommunicationError()
{
    const auto last_update_ns = last_update_time_ns_.load(std::memory_order_relaxed);

    if (last_update_ns == 0) {
        // No telemetry callback has fired yet since construction - treat as a comm error.
        return true;
    }

    const auto last_update = std::chrono::steady_clock::time_point(
        std::chrono::nanoseconds(last_update_ns));

    return isCommTimedOut(last_update, std::chrono::steady_clock::now(), comm_timeout_);
}

bool PhidgetMotorDriver::isCommTimedOut(
    const std::chrono::steady_clock::time_point & last_update,
    const std::chrono::steady_clock::time_point & now,
    const std::chrono::nanoseconds & timeout)
{
    return (now - last_update) > timeout;
}

void CCONV PhidgetMotorDriver::positionChangeHandler(
    PhidgetEncoderHandle phid,
    void *ctx,
    int positionChange,
    double timeChange,
    int indexTriggered)
{
    (void)positionChange;
    (void)phid;
    (void)timeChange;
    (void)indexTriggered;

    PhidgetMotorDriver * driver = static_cast<PhidgetMotorDriver*>(ctx);

    driver->last_update_time_ns_.store(
        std::chrono::steady_clock::now().time_since_epoch().count(),
        std::memory_order_relaxed);

    int64_t position;

    PhidgetReturnCode ret = PhidgetEncoder_getPosition(phid, &position);

    if (ret == EPHIDGET_OK) {

        if (driver->direction_reversed_ == true) {
            driver->encoder_ticks_ = position * (-1);
        } else {
            driver->encoder_ticks_ = position;
        }

        driver->position_time_change_ = timeChange / 1000.0;

        const double vel = encoderCountsToMotorRpm(
            driver->encoder_ticks_ - driver->prev_encoder_ticks_,
            driver->position_time_change_, driver->encoder_resolution_);
        const int64_t pos = driver->encoder_ticks_ / 4;

        {
            std::lock_guard<std::mutex> lck(driver->state_mtx_);
            driver->state_.vel = vel;
            driver->state_.pos = pos;
        }

        driver->prev_encoder_ticks_ = driver->encoder_ticks_;

        driver->last_encoder_event_ns_.store(
            std::chrono::steady_clock::now().time_since_epoch().count(),
            std::memory_order_relaxed);
    }
}

void CCONV PhidgetMotorDriver::currentChangeHandler(
    PhidgetCurrentInputHandle phid,
    void * ctx,
    double current
)
{
    (void)phid;

    PhidgetMotorDriver * driver = static_cast<PhidgetMotorDriver*>(ctx);

    driver->last_update_time_ns_.store(
        std::chrono::steady_clock::now().time_since_epoch().count(),
        std::memory_order_relaxed);

    {
        std::lock_guard<std::mutex> lck(driver->state_mtx_);
        driver->state_.current = static_cast<int16_t>(current * 1000.0);
    }
}

void CCONV PhidgetMotorDriver::temperatureChangeHandler(
    PhidgetTemperatureSensorHandle phid,
    void *ctx,
    double temperature
)
{
    (void)phid;

    PhidgetMotorDriver * driver = static_cast<PhidgetMotorDriver*>(ctx);

    driver->last_update_time_ns_.store(
        std::chrono::steady_clock::now().time_since_epoch().count(),
        std::memory_order_relaxed);

    std::lock_guard<std::mutex> lck(driver->state_mtx_);
    driver->state_.temp = static_cast<float>(temperature);
}

void CCONV PhidgetMotorDriver::setTargetVelocityHandler(
    PhidgetHandle phid,
    void *ctx,
    PhidgetReturnCode res)
{
    (void)phid;

    PhidgetMotorDriver * driver = static_cast<PhidgetMotorDriver*>(ctx);

    driver->set_speed_pending_ = false;

    if (isFailsafeTrippedReturnCode(res)) {
        // Written from the Phidget SDK's own callback thread, same cross-thread pattern as
        // last_update_time_ns_ above - read (RT thread) via isFailsafeTripped(). Only cleared by
        // resetFailsafe(), never automatically here, so a trip stays latched even once commands
        // start succeeding again after a re-arm.
        driver->failsafe_tripped_.store(true, std::memory_order_relaxed);
    }
}

bool PhidgetMotorDriver::isFailsafeTrippedReturnCode(const PhidgetReturnCode res)
{
    return res == EPHIDGET_FAILSAFE;
}

PhidgetMotorDriver::FailsafeAction PhidgetMotorDriver::selectFailsafeAction(
    const bool enabled, const bool tripped)
{
    if (tripped) {
        return FailsafeAction::kReopenAndEnable;
    }

    return enabled ? FailsafeAction::kFeed : FailsafeAction::kEnable;
}

void PhidgetMotorDriver::armFailsafe()
{
    std::lock_guard<std::mutex> lck(failsafe_mtx_);

    switch (selectFailsafeAction(failsafe_enabled_, isFailsafeTripped())) {
        case FailsafeAction::kEnable:
            enableFailsafe();
            break;

        case FailsafeAction::kFeed: {
            // Already armed (e.g. on_activate() after a deactivate): enabling again isn't allowed
            // on an open channel, so just feed the timer. If that fails the channel has most
            // likely tripped without a command noticing yet - recover it the same way.
            const PhidgetReturnCode ret = PhidgetDCMotor_resetFailsafe(motor_handle_);

            if (ret != EPHIDGET_OK) {
                RCLCPP_WARN_STREAM(
                    logger_, "PhidgetDCMotor_resetFailsafe() returned " << returnCodeToString(ret)
                        << " for motor channel " << static_cast<int>(channel_)
                        << "; re-opening the channel and re-arming failsafe.");
                reopenMotorChannel();
                enableFailsafe();
            }
            break;
        }

        case FailsafeAction::kReopenAndEnable:
            reopenMotorChannel();
            enableFailsafe();
            break;
    }

    failsafe_tripped_.store(false, std::memory_order_relaxed);
}

void PhidgetMotorDriver::resetFailsafe()
{
    // A tripped Phidget channel rejects every further call - PhidgetDCMotor_resetFailsafe() and
    // PhidgetDCMotor_enableFailsafe() included - until it is closed and re-opened, so the only way
    // to un-stick it is armFailsafe()'s re-open path. For a healthy channel, armFailsafe() just
    // feeds the timer (or enables it, if it never was). Throws std::runtime_error on failure, in
    // which case failsafe_tripped_ stays set and the caller keeps motion refused.
    armFailsafe();
}

bool PhidgetMotorDriver::isFailsafeTripped()
{
    return failsafe_tripped_.load(std::memory_order_relaxed);
}

void PhidgetMotorDriver::sendCmdVel(const float cmd)
{
    if (auto driver = driver_.lock()) {
        // `driver` is intentionally unused below - this is purely a lifetime guard, holding the
        // driver alive for the duration of this call, not a handle we dereference.

        // A previous async PhidgetDCMotor_setTargetVelocity_async() call hasn't completed yet -
        // this command is dropped rather than queued. At the 100 Hz write() rate the next cycle's
        // command supersedes it almost immediately, so this is intentional, not an oversight; it
        // is not currently surfaced as an error/counter to the caller.
        if (set_speed_pending_) return;

        float cmd_temp = 0.0f;

        if (direction_reversed_) {
            cmd_temp = cmd * (-1.0);
        } else {
            cmd_temp = cmd;
        }

        PhidgetDCMotor_setTargetVelocity_async(
            motor_handle_,
            cmd_temp,
            PhidgetMotorDriver::setTargetVelocityHandler,
            this);

        set_speed_pending_ = true;
    }
}

}  // namespace rover_hardware_interface
