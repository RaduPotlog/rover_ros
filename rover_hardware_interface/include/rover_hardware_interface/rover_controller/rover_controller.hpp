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

#ifndef ROVER_HARDWARE_INTERFACE_ROVER_CONTROLLER_ROVER_CONTROLLER_HPP_
#define ROVER_HARDWARE_INTERFACE_ROVER_CONTROLLER_ROVER_CONTROLLER_HPP_

#include <atomic>
#include <functional>
#include <memory>
#include <thread>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>

#include "rover_hardware_interface/rover_controller/rover_controller_types.hpp"
#include "rover_hardware_interface/rover_modbus/modbus.hpp"
#include "rover_hardware_interface/utils.hpp"

namespace rover_hardware_interface
{

class ContactCoilHandler
{

public:

    ContactCoilHandler(std::shared_ptr<RoverModbus> rover_modbus);

    ~ContactCoilHandler();

    bool start();

    bool isContactCoilHandlerEnabled() const;

    // SW E-STOP USER BTN - sw_e_stop_user_button
    void eStopUserBtnTrigger(const bool state);
    
    // SW E-STOP MOTOR DRIVER FAULT - sw_e_stop_motor_driver_fault
    void eStopMotorDriverFaultTrigger(const bool state);
    
    // SW E-STOP LATCH RESET - sw_e_stop_latch_reset
    void eStopLatchReset();

    std::unordered_map<RoverControllerGpio, bool> getIoState();

private:
    
    void initCoils();
    
    bool readDiscreteContact(const ContactInfo &contact);

    bool readDiscreteCoil(const CoilInfo &coil);

    std::unordered_map<RoverControllerGpio, bool> queryControlInterfaceIOStates();

    void contactCoilHandlerThread();

    std::thread contact_coil_handler_thread_;
    std::atomic_bool contact_coil_handler_enabled_ = false;

    std::shared_ptr<RoverModbus> rover_modbus_;

    static const std::vector<RoverControllerContactInfo> contacts_config_info_storage_;
    static const std::vector<RoverControllerCoilInfo> coils_config_info_storage_;

    std::mutex write_to_modbus_mtx_;

    std::unordered_map<RoverControllerGpio, bool> io_state_;

    rclcpp::Logger logger_{rclcpp::get_logger("RoverSystem")};
};

class RoverController
{

public:
  
    RoverController();

    // Start resources and ContactCoilHandler thread
    void start();

    // SW E-STOP USER BTN - sw_e_stop_user_button
    void eStopUserBtnTrigger(const bool state);
    
    // SW E-STOP MOTOR DRIVER FAULT - sw_e_stop_motor_driver_fault
    void eStopMotorDriverFaultTrigger(const bool state);
    
    // SW E-STOP LATCH RESET - sw_e_stop_latch_reset
    void eStopLatchReset();

    std::unordered_map<RoverControllerGpio, bool> queryControlInterfaceIOStates() const;

    bool isPinActive(const RoverControllerGpio pin);

private:
    
    std::unique_ptr<ContactCoilHandler> contactCoilHandler_;

    bool waitFor(std::chrono::milliseconds timeout);

    std::shared_ptr<RoverModbus> rover_modbus_;

    std::mutex e_stop_cv_mtx_;
    
    std::condition_variable e_stop_cv_;
    
    volatile std::atomic_bool should_abort_e_stop_reset_ = false;
};

}  // namespace rover_hardware_interface

#endif  // ROVER_HARDWARE_INTERFACE_ROVER_CONTROLLER_ROVER_CONTROLLER_HPP_
