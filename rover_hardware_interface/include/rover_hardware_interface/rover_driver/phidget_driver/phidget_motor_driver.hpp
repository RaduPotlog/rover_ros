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

#ifndef ROVER_HARDWARE_INTERFACES_ROVER_DRIVER_PHIDGET_MOTOR_DRIVER_PHIDGET_MOTOR_DRIVER_HPP_
#define ROVER_HARDWARE_INTERFACES_ROVER_DRIVER_PHIDGET_MOTOR_DRIVER_PHIDGET_MOTOR_DRIVER_HPP_

#include <functional>
#include <vector>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <future>
#include <memory>
#include <mutex>
#include <string>

#include <libphidget22/phidget22.h>

#include <rclcpp/rclcpp.hpp>

#include "rover_hardware_interface/rover_driver/driver.hpp"
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

private:

    std::mutex init_mtx_;
    std::promise<void> init_promise_;

    std::unordered_map<MotorNames, std::shared_ptr<MotorDriverInterface>> motor_drivers_;

    rclcpp::Logger logger_{rclcpp::get_logger("RoverSystem")};
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

private:
        
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
    
    double calculateRPM(int64_t delta_ticks, double dt, float ppr);

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

    bool direction_reversed_;

    std::atomic<bool> set_speed_done_{false}; 

    rclcpp::Logger logger_{rclcpp::get_logger("RoverSystem")};
};

} // namespace rover_hardware_interface

#endif  // ROVER_HARDWARE_INTERFACES_ROVER_DRIVER_PHIDGET_MOTOR_DRIVER_PHIDGET_MOTOR_DRIVER_HPP_