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

#include "rover_hardware_interface/rover_controller/rover_controller.hpp"

#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <thread>
#include <utility>

#include "rover_hardware_interface/rover_controller/rover_controller_types.hpp"

namespace rover_hardware_interface
{

const std::vector<RoverControllerContactInfo> ContactCoilHandler::contacts_config_info_storage_ = {
    
    RoverControllerContactInfo {
        RoverControllerGpio {RoverControllerGpio::GPIO_HW_E_STOP_USER_BTN},
        ContactInfo { Contact::CONTACT_0 },
    }
};

const std::vector<RoverControllerCoilInfo> ContactCoilHandler::coils_config_info_storage_ = {

    RoverControllerCoilInfo { 
        RoverControllerGpio {RoverControllerGpio::GPIO_MOTOR_CONTACTOR_ENGAGED},
        CoilInfo { Coil::COIL_0, false, false},    
    },

    RoverControllerCoilInfo { 
        RoverControllerGpio {RoverControllerGpio::GPIO_SW_E_STOP_CPU_WDG_TRIGGER},
        CoilInfo { Coil::COIL_1, true, true},
    },
    
    RoverControllerCoilInfo { 
        RoverControllerGpio {RoverControllerGpio::GPIO_SW_E_STOP_USER_BUTTON},
        CoilInfo { Coil::COIL_2, true, true},    
    },
    
    RoverControllerCoilInfo { 
        RoverControllerGpio {RoverControllerGpio::GPIO_SW_E_STOP_MOTOR_DRIVER_FAULT},
        CoilInfo { Coil::COIL_3, true, true},    
    },

    RoverControllerCoilInfo { 
        RoverControllerGpio {RoverControllerGpio::GPIO_SW_E_STOP_LATCH_RESET},
        CoilInfo { Coil::COIL_4, false, true},    
    },

    RoverControllerCoilInfo { 
        RoverControllerGpio {RoverControllerGpio::GPIO_SW_E_STOP_LATCH_STATUS},
        CoilInfo { Coil::COIL_5, false, false},    
    },
};

ContactCoilHandler::ContactCoilHandler(std::shared_ptr<RoverModbus> rover_modbus)
: rover_modbus_(rover_modbus)
{

}

ContactCoilHandler::~ContactCoilHandler()
{ 
    if (isContactCoilHandlerEnabled()) {
        contact_coil_handler_enabled_ = false;
        contact_coil_handler_thread_.join();
    }
}

bool ContactCoilHandler::start()
{
    if (isContactCoilHandlerEnabled()) {
        return true;
    }
    
    initCoils();

    contact_coil_handler_enabled_ = true;
    contact_coil_handler_thread_ = std::thread(&ContactCoilHandler::contactCoilHandlerThread, this);

    return isContactCoilHandlerEnabled();
}

bool ContactCoilHandler::isContactCoilHandlerEnabled() const 
{ 
    return contact_coil_handler_thread_.joinable(); 
}

// SW E-STOP USER BTN - sw_e_stop_user_button
void ContactCoilHandler::eStopUserBtnTrigger(const bool state)
{
    write_to_modbus_mtx_.lock();
    rover_modbus_->writeDiscreteCoil(coils_config_info_storage_[2].coil_info, state);
    write_to_modbus_mtx_.unlock();
}

// SW E-STOP MOTOR DRIVER FAULT - sw_e_stop_motor_driver_fault
void ContactCoilHandler::eStopMotorDriverFaultTrigger(const bool state)
{
    write_to_modbus_mtx_.lock();
    rover_modbus_->writeDiscreteCoil(coils_config_info_storage_[3].coil_info, state);
    write_to_modbus_mtx_.unlock();
}

// SW E-STOP LATCH RESET - sw_e_stop_latch_reset
void ContactCoilHandler::eStopLatchReset()
{
    write_to_modbus_mtx_.lock();
    rover_modbus_->writeDiscreteCoil(coils_config_info_storage_[4].coil_info, true);
    rover_modbus_->writeDiscreteCoil(coils_config_info_storage_[4].coil_info, false);
    write_to_modbus_mtx_.unlock();
}

std::unordered_map<RoverControllerGpio, bool> ContactCoilHandler::getIoState()
{
    std::unordered_map<RoverControllerGpio, bool> io_state;

    if (write_to_modbus_mtx_.try_lock()) {
        std::lock_guard<std::mutex> e_stop_lck(write_to_modbus_mtx_, std::adopt_lock);
        io_state = io_state_;
    }

    return io_state;
}

void ContactCoilHandler::initCoils()
{
    for (size_t i = 0; i < coils_config_info_storage_.size(); i++) {
        rover_modbus_->writeDiscreteCoil(coils_config_info_storage_[i].coil_info, coils_config_info_storage_[i].coil_info.default_coil_state);
    }
}

bool ContactCoilHandler::readDiscreteContact(const ContactInfo &contact)
{
    uint16_t contact_state = rover_modbus_->readDiscreteContact(contact);
    
    return (contact_state == 255 ? false : (contact_state & 0xFFU));
}

bool ContactCoilHandler::readDiscreteCoil(const CoilInfo &coil)
{
    uint16_t conil_state = rover_modbus_->readDiscreteCoil(coil);
    
    return (conil_state == 255 ? false : (conil_state & 0xFFU));
}

std::unordered_map<RoverControllerGpio, bool> ContactCoilHandler::queryControlInterfaceIOStates()
{
    std::unordered_map<RoverControllerGpio, bool> io_state;

    std::for_each(contacts_config_info_storage_.begin(), contacts_config_info_storage_.end(), [&](RoverControllerContactInfo contact) {
        bool is_active = readDiscreteContact(contact.contact_info);
        io_state.emplace(static_cast<RoverControllerGpio>(contact.pin), is_active);
    });

    std::for_each(coils_config_info_storage_.begin(), coils_config_info_storage_.end(), [&](RoverControllerCoilInfo coil) {
        bool is_active = readDiscreteCoil(coil.coil_info);
        io_state.emplace(static_cast<RoverControllerGpio>(coil.pin), is_active);
    });

    return io_state;
}

void ContactCoilHandler::contactCoilHandlerThread()
{
    static bool wdg_state = false;

    while (contact_coil_handler_enabled_) {
        // Trigger watchdog
        write_to_modbus_mtx_.lock();
        io_state_ = queryControlInterfaceIOStates();
        rover_modbus_->writeDiscreteCoil(coils_config_info_storage_[1].coil_info, wdg_state);
        wdg_state = !wdg_state;
        write_to_modbus_mtx_.unlock();  
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}

RoverController::RoverController()
{
    rover_modbus_ = std::make_shared<RoverModbus>("192.168.88.11", 502);
}

void RoverController::start()
{
    contactCoilHandler_ = std::make_unique<ContactCoilHandler>(rover_modbus_);
    contactCoilHandler_->start();
}

// SW E-STOP USER BTN - sw_e_stop_user_button
void RoverController::eStopUserBtnTrigger(const bool state)
{
    if (!contactCoilHandler_->isContactCoilHandlerEnabled()) {
        return;
    }

    contactCoilHandler_->eStopUserBtnTrigger(state);
}

// SW E-STOP MOTOR DRIVER FAULT - sw_e_stop_motor_driver_fault
void RoverController::eStopMotorDriverFaultTrigger(const bool state)
{
    if (!contactCoilHandler_->isContactCoilHandlerEnabled()) {
        return;
    }

    contactCoilHandler_->eStopMotorDriverFaultTrigger(state);
}

// SW E-STOP LATCH RESET - sw_e_stop_latch_reset
void RoverController::eStopLatchReset()
{
    if (!contactCoilHandler_->isContactCoilHandlerEnabled()) {
        return;
    }

    contactCoilHandler_->eStopLatchReset();
}

std::unordered_map<RoverControllerGpio, bool> RoverController::queryControlInterfaceIOStates() const
{
    std::unordered_map<RoverControllerGpio, bool> io_state;

    if (!contactCoilHandler_->isContactCoilHandlerEnabled()) {
        return io_state;
    }

    io_state = contactCoilHandler_->getIoState();

    return io_state;
}

bool RoverController::isPinActive(const RoverControllerGpio pin)
{
    std::unordered_map<RoverControllerGpio, bool> io_state;

    if (!contactCoilHandler_->isContactCoilHandlerEnabled()) {
        return false;
    }

    io_state = contactCoilHandler_->getIoState();

    return (io_state[pin] == true);
}

bool RoverController::waitFor(std::chrono::milliseconds timeout)
{
    std::unique_lock<std::mutex> lck(e_stop_cv_mtx_);

    should_abort_e_stop_reset_ = false;
    
    bool interrupted = e_stop_cv_.wait_for(
        lck, timeout, [&]() { 
            return should_abort_e_stop_reset_;
    });

    return !interrupted;
}

}  // namespace rover_hardware_interface
