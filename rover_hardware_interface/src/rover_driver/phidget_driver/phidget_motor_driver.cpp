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

#include <iostream>

#include "rover_hardware_interface/rover_driver/phidget_driver/phidget_motor_driver.hpp"

#include "rover_hardware_interface/rover_driver/driver.hpp"
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
            std::cerr << "An exception occurred while setting init promise: " << e.what() << std::endl;
        }
    }
    
    return future;
}

DriverState PhidgetDriver::readState()
{
    DriverState driver_state;
    
    timespec current_time;
    clock_gettime(CLOCK_MONOTONIC, &current_time);

    const auto motor_state = this->getMotorDriver(MotorNames::DEFAULT)->readState();
    driver_state.fault_flags = 0;
    driver_state.runtime_stat_flag = 0;
    driver_state.driver_current = motor_state.current;
    driver_state.temp = motor_state.temp;

    return driver_state;
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
{
    encoder_resolution_ = drivetrain_settings.encoder_resolution;

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
	
    // Set acceleration
    ret = PhidgetDCMotor_setAcceleration(motor_handle_, 1.0f);

    if (ret != EPHIDGET_OK) {
        throw std::runtime_error("Failed to set acceleration for motor channel " + 
            std::to_string(channel_));
    }

    // Set current limit
    ret = PhidgetDCMotor_setCurrentLimit(motor_handle_, 15.0);

    if (ret != EPHIDGET_OK) {
        throw std::runtime_error("Failed to set current limit for motor channel " + 
            std::to_string(channel_));
    }

    /*
     *  CurrentRegulatorGain = CurrentLimit * (Voltage / 12)
     */
    ret = PhidgetDCMotor_setCurrentRegulatorGain(motor_handle_, 15.0);

    if (ret != EPHIDGET_OK) {
        throw std::runtime_error("Failed to set current regulator gain for motor channel " + 
            std::to_string(channel_));
    }

    // Enable fail safe
    ret = PhidgetDCMotor_enableFailsafe(motor_handle_, 5000);

    if (ret != EPHIDGET_OK) {
        throw std::runtime_error("Failed to enable fail safe mode for motor channel " + 
            std::to_string(channel_));
    }

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
}

MotorDriverState PhidgetMotorDriver::readState()
{
    return state_;
}

double PhidgetMotorDriver::calculateRPM(int64_t delta_ticks, double dt, float ppr)
{
    return ((static_cast<double>(delta_ticks) / static_cast<double>(ppr)) * (60.0f / dt));
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
    
    int64_t position;

	PhidgetReturnCode ret = PhidgetEncoder_getPosition(phid, &position);

    if (ret == EPHIDGET_OK) {
        
        if (driver->direction_reversed_ == true) {
            driver->encoder_ticks_ = position * (-1);        
        }
        else {
            driver->encoder_ticks_ = position;
        }

        driver->position_time_change_ = timeChange / 1000.0f;
        
        int64_t delta_encoder_ticks = driver->encoder_ticks_ - driver->prev_encoder_ticks_;

        driver->state_.vel = static_cast<int16_t>(driver->calculateRPM(delta_encoder_ticks, driver->position_time_change_, driver->encoder_resolution_) / 8.0);
        driver->state_.pos = driver->encoder_ticks_;
        
        driver->prev_encoder_ticks_ = driver->encoder_ticks_;
        
        //RCLCPP_INFO(driver->logger_, "Encoder position changed channel = %d, position = %ld, velocity = %d, time = %lf", driver->channel_, driver->state_.pos, driver->state_.vel, driver->position_time_change_);
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

    driver->state_.current = static_cast<int16_t>(current * 1000.0);

    // RCLCPP_INFO(driver->logger_, "Motor current changed channel = %d, current = %f", driver->channel_, current);
}

void CCONV PhidgetMotorDriver::temperatureChangeHandler(
    PhidgetTemperatureSensorHandle phid,
    void *ctx,
    double temperature
)
{
    (void)phid;

    PhidgetMotorDriver * driver = static_cast<PhidgetMotorDriver*>(ctx);

    driver->state_.temp = static_cast<float>(temperature);
}

void CCONV PhidgetMotorDriver::setTargetVelocityHandler(
    PhidgetHandle phid, 
    void *ctx, 
    PhidgetReturnCode res) 
{
    (void)phid;
    (void)res;

    PhidgetMotorDriver * driver = static_cast<PhidgetMotorDriver*>(ctx);

    driver->set_speed_done_ = false;
}

void PhidgetMotorDriver::sendCmdVel(const float cmd)
{
    if (auto driver = driver_.lock()) {

        if (set_speed_done_) return;
        
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
        
        set_speed_done_ = true;
    }
}

}  // namespace rover_hardware_interface