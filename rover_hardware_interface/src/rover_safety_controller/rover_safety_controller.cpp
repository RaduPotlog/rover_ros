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

#include "rover_hardware_interface/rover_safety_controller/rover_safety_controller.hpp"

#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <thread>
#include <utility>

#include "rover_hardware_interface/rover_safety_controller/rover_safety_controller_types.hpp"

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

ContactCoilHandler::ContactCoilHandler(std::shared_ptr<RoverModbusInterface> rover_modbus)
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
    std::lock_guard<std::mutex> lck(write_to_modbus_mtx_);
    rover_modbus_->writeDiscreteCoil(coils_config_info_storage_[2].coil_info, state);
}

// SW E-STOP MOTOR DRIVER FAULT - sw_e_stop_motor_driver_fault
void ContactCoilHandler::eStopMotorDriverFaultTrigger(const bool state)
{
    std::lock_guard<std::mutex> lck(write_to_modbus_mtx_);
    rover_modbus_->writeDiscreteCoil(coils_config_info_storage_[3].coil_info, state);
}

// SW E-STOP LATCH RESET - sw_e_stop_latch_reset
void ContactCoilHandler::eStopLatchReset()
{
    std::lock_guard<std::mutex> lck(write_to_modbus_mtx_);
    rover_modbus_->writeDiscreteCoil(coils_config_info_storage_[4].coil_info, true);
    rover_modbus_->writeDiscreteCoil(coils_config_info_storage_[4].coil_info, false);
}

void ContactCoilHandler::getIoState(std::unordered_map<RoverControllerGpio, bool> & io_state)
{
    if (write_to_modbus_mtx_.try_lock()) {
        std::lock_guard<std::mutex> e_stop_lck(write_to_modbus_mtx_, std::adopt_lock);
        io_state = io_state_;
    }
    // else: leave `io_state` as-is (the caller's last-known-good values) rather than clobber it
    // with an empty map on contention.
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
        {
            // Trigger watchdog
            std::lock_guard<std::mutex> lck(write_to_modbus_mtx_);
            io_state_ = queryControlInterfaceIOStates();
            rover_modbus_->writeDiscreteCoil(coils_config_info_storage_[1].coil_info, wdg_state);
            wdg_state = !wdg_state;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}

RoverSafetyController::RoverSafetyController(
    const std::string & modbus_host, const int modbus_port,
    const unsigned modbus_connection_retry_count,
    const std::chrono::milliseconds modbus_connection_retry_delay)
{
    rover_modbus_ = std::make_shared<RoverModbus>(
        modbus_host, modbus_port, modbus_connection_retry_count, modbus_connection_retry_delay);
}

RoverSafetyController::RoverSafetyController(std::shared_ptr<RoverModbusInterface> rover_modbus)
: rover_modbus_(std::move(rover_modbus))
{

}

void RoverSafetyController::start()
{
    contactCoilHandler_ = std::make_unique<ContactCoilHandler>(rover_modbus_);
    contactCoilHandler_->start();
}

// SW E-STOP USER BTN - sw_e_stop_user_button
void RoverSafetyController::eStopUserBtnTrigger(const bool state)
{
    // contactCoilHandler_ is null until start() is called - guard against that first (rather
    // than dereferencing it to ask isContactCoilHandlerEnabled()) so calling this before start()
    // is a safe no-op instead of a null-pointer dereference.
    if (!contactCoilHandler_ || !contactCoilHandler_->isContactCoilHandlerEnabled()) {
        return;
    }

    contactCoilHandler_->eStopUserBtnTrigger(state);
}

// SW E-STOP MOTOR DRIVER FAULT - sw_e_stop_motor_driver_fault
void RoverSafetyController::eStopMotorDriverFaultTrigger(const bool state)
{
    if (!contactCoilHandler_ || !contactCoilHandler_->isContactCoilHandlerEnabled()) {
        return;
    }

    contactCoilHandler_->eStopMotorDriverFaultTrigger(state);
}

// SW E-STOP LATCH RESET - sw_e_stop_latch_reset
void RoverSafetyController::eStopLatchReset()
{
    if (!contactCoilHandler_ || !contactCoilHandler_->isContactCoilHandlerEnabled()) {
        return;
    }

    contactCoilHandler_->eStopLatchReset();
}

const std::unordered_map<RoverControllerGpio, bool> & RoverSafetyController::queryControlInterfaceIOStates()
{
    if (contactCoilHandler_ && contactCoilHandler_->isContactCoilHandlerEnabled()) {
        contactCoilHandler_->getIoState(io_state_cache_);
    }

    return io_state_cache_;
}

bool RoverSafetyController::isPinActive(const RoverControllerGpio pin)
{
    const auto & io_state = queryControlInterfaceIOStates();
    const auto it = io_state.find(pin);

    return it != io_state.end() && it->second;
}

}  // namespace rover_hardware_interface
