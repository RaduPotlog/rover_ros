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

#include "rover_hardware_interface/rover_system/rover_system.hpp"

#include <array>
#include <chrono>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <rclcpp/logging.hpp>

#include "rover_hardware_interface/rover_driver/rover_a1_driver.hpp"
#include "rover_hardware_interface/system_ros_interface/system_ros_interface.hpp"

#include "rover_hardware_interface/utils.hpp"

namespace rover_hardware_interface
{

RoverSystem::RoverSystem(const std::vector<std::string> & joint_order)
: SystemInterface()
, joint_size_(joint_order.size())
, joint_order_(joint_order) 
, joints_names_sorted_(joint_size_)
{

}

CallbackReturn RoverSystem::on_init(const hardware_interface::HardwareComponentInterfaceParams & params)
{
    if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }

    try {
        checkJointSize();
        sortAndCheckJointNames();
        setInitialValues();
        checkInterfaces();
    } catch (const std::runtime_error & e) {
        RCLCPP_ERROR_STREAM(logger_, "An exception occurred while initializing: " << e.what());
        return CallbackReturn::ERROR;
    }

    try {
        readDrivetrainSettings();
        readDriverStatesUpdateFrequency();
        readDriverInitAndActivationAttempts();
    } catch (const std::invalid_argument & e) {
        RCLCPP_ERROR_STREAM(logger_, "An exception occurred while reading the parameters: " << e.what());
        return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
}

CallbackReturn RoverSystem::on_configure(const rclcpp_lifecycle::State &)
{
    try {
        configureRoverController();
        configureRoverDriver();
        configureEStop();
    } catch (const std::runtime_error & e) {
        RCLCPP_ERROR_STREAM(logger_, "Failed to initialize motors controllers. Error: " << e.what());
        return CallbackReturn::ERROR;
    }

    std::fill(hw_commands_velocities_.begin(), hw_commands_velocities_.end(), 0.0);
    std::fill(hw_states_positions_.begin(), hw_states_positions_.end(), 0.0);
    std::fill(hw_states_velocities_.begin(), hw_states_velocities_.end(), 0.0);
    std::fill(hw_states_efforts_.begin(), hw_states_efforts_.end(), 0.0);

    if (!operationWithAttempts(std::bind(&RoverDriverInterface::activate, rover_driver_), 
        max_rover_driver_activation_attempts_)) {
            RCLCPP_ERROR_STREAM(logger_, "Failed to activate Rover System: Couldn't activate RoverDriver in " 
                                         << max_rover_driver_activation_attempts_ << " attempts.");
        return CallbackReturn::ERROR;
    }

    system_ros_interface_ = std::make_unique<SystemROSInterface>("hardware_controller");

    system_ros_interface_->addService<TriggerSrv, std::function<void()>>(
        "hardware_interface/sw_user_e_stop_set", std::bind(&EmergencyStopInterface::setEStop, e_stop_), 1,
        rclcpp::CallbackGroupType::MutuallyExclusive);
    
    auto e_stop_reset_qos = rclcpp::ServicesQoS();
    e_stop_reset_qos.keep_last(1);
    system_ros_interface_->addService<TriggerSrv, std::function<void()>>(
        "hardware_interface/sw_user_e_stop_reset", std::bind(&RoverSystem::resetEStop, this), 2,
        rclcpp::CallbackGroupType::MutuallyExclusive, e_stop_reset_qos);

    auto e_stop_latch_reset_qos = rclcpp::ServicesQoS();
    e_stop_latch_reset_qos.keep_last(1);
    system_ros_interface_->addService<TriggerSrv, std::function<void()>>(
        "hardware_interface/sw_e_stop_latch_reset", std::bind(&RoverSystem::resetEStopLatch, this), 2,
        rclcpp::CallbackGroupType::MutuallyExclusive, e_stop_latch_reset_qos);

    const auto gpio_state = rover_controller_->queryControlInterfaceIOStates();
    system_ros_interface_->updateMsgGpioStates(gpio_state);
    system_ros_interface_->publishGpioStateMsg();

    return CallbackReturn::SUCCESS;
}

CallbackReturn RoverSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
    rover_controller_.reset();
    rover_driver_->deinitialize();
    rover_driver_.reset();
    e_stop_.reset();
    system_ros_interface_.reset();
    
    return CallbackReturn::SUCCESS;
}

CallbackReturn RoverSystem::on_activate(const rclcpp_lifecycle::State &)
{
    return CallbackReturn::SUCCESS;
}

CallbackReturn RoverSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
    return CallbackReturn::SUCCESS;
}

CallbackReturn RoverSystem::on_shutdown(const rclcpp_lifecycle::State &)
{
    rover_controller_.reset();
    rover_driver_->deinitialize();
    rover_driver_.reset();
    e_stop_.reset();
    system_ros_interface_.reset();
    
    return CallbackReturn::SUCCESS;
}

CallbackReturn RoverSystem::on_error(const rclcpp_lifecycle::State &)
{
    if (system_ros_interface_) {
        system_ros_interface_->broadcastOnDiagnosticTasks(
            diagnostic_msgs::msg::DiagnosticStatus::ERROR,
            "An error has occurred during a node state transition.");

        system_ros_interface_.reset();
    }
    
    rover_driver_->deinitialize();
    rover_driver_.reset();

    return CallbackReturn::SUCCESS;
}

std::vector<StateInterface> RoverSystem::export_state_interfaces()
{
    std::vector<StateInterface> state_interfaces;

    for (std::size_t i = 0; i < joint_size_; i++) {
        state_interfaces.emplace_back(StateInterface(
            joints_names_sorted_[i], hardware_interface::HW_IF_POSITION, &hw_states_positions_[i]));
        state_interfaces.emplace_back(StateInterface(
            joints_names_sorted_[i], hardware_interface::HW_IF_VELOCITY, &hw_states_velocities_[i]));
        state_interfaces.emplace_back(StateInterface(
            joints_names_sorted_[i], hardware_interface::HW_IF_EFFORT, &hw_states_efforts_[i]));
    }

    return state_interfaces;
}

std::vector<CommandInterface> RoverSystem::export_command_interfaces()
{
    std::vector<CommandInterface> command_interfaces;
    
    for (std::size_t i = 0; i < joint_size_; i++) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            joints_names_sorted_[i], hardware_interface::HW_IF_VELOCITY, &hw_commands_velocities_[i]));
    }

    return command_interfaces;
}

return_type RoverSystem::read(const rclcpp::Time & time, const rclcpp::Duration & /* period */)
{
    updateMotorsState(time);

    if (time >= next_driver_state_update_time_) {
        updateDriverState();
        updateDriverStateMsg();
        system_ros_interface_->publishRobotDriverState();
        
        const auto gpio_state = rover_controller_->queryControlInterfaceIOStates();
        system_ros_interface_->updateMsgGpioStates(gpio_state);
        system_ros_interface_->publishGpioStateMsg();
    
        next_driver_state_update_time_ = time + driver_states_update_period_;
    }
    
    return return_type::OK;
}

return_type RoverSystem::write(const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
{
    const auto lifecycle_state = this->get_lifecycle_state().id();
    
    if (lifecycle_state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        handleRoverDriverWriteOperation([this] {
            const auto speed_cmds = getSpeedCmd();
            rover_driver_->sendSpeedCmd(speed_cmds);
        });
    }

    return return_type::OK;
}

void RoverSystem::checkJointSize() const
{
    if (info_.joints.size() != joint_size_) {
        throw std::runtime_error(
            "Wrong number of joints defined: " + std::to_string(info_.joints.size()) + ", " +
            std::to_string(joint_size_) + " expected.");
    }
}

void RoverSystem::sortAndCheckJointNames()
{
    for (std::size_t i = 0; i < joint_size_; i++) {
        std::size_t match_count = 0;

        for (std::size_t j = 0; j < joint_size_; j++) {
            if (checkIfJointNameContainValidSequence(info_.joints[j].name, joint_order_[i])) {
                joints_names_sorted_[i] = info_.joints[j].name;
                ++match_count;
            }
        }

        if (match_count != 1) {
            throw std::runtime_error(
                "There should be exactly one joint containing " + joint_order_[i] + ", " +
                std::to_string(match_count) + " found.");
        }
    }
}

void RoverSystem::setInitialValues()
{
    hw_commands_velocities_.resize(joint_size_, 0.0);

    hw_states_positions_.resize(joint_size_, std::numeric_limits<double>::quiet_NaN());
    hw_states_velocities_.resize(joint_size_, std::numeric_limits<double>::quiet_NaN());
    hw_states_efforts_.resize(joint_size_, std::numeric_limits<double>::quiet_NaN());
}

void RoverSystem::checkInterfaces() const
{
    for (const hardware_interface::ComponentInfo & joint : info_.joints) {
        // Commands
        if (joint.command_interfaces.size() != 1) {
            throw std::runtime_error(
                "Joint " + joint.name + " has " + std::to_string(joint.command_interfaces.size()) +
                " command interfaces. 1 expected.");
        }

        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
            throw std::runtime_error(
                "Joint " + joint.name + " has " + joint.command_interfaces[0].name +
                " command interface. " + hardware_interface::HW_IF_VELOCITY + " expected.");
        }

        // States
        if (joint.state_interfaces.size() != 3) {
            throw std::runtime_error(
                "Joint " + joint.name + " has " + std::to_string(joint.state_interfaces.size()) +
                " state  " + (joint.state_interfaces.size() == 1 ? "interface." : "interfaces.") +
                " 3 expected.");
        }

        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            throw std::runtime_error(
                "Joint " + joint.name + " has " + joint.state_interfaces[0].name +
                " as first state interface. " + hardware_interface::HW_IF_POSITION + " expected.");
        }

        if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
            throw std::runtime_error(
                "Joint " + joint.name + " has " + joint.state_interfaces[1].name +
                " as second state interface. " + hardware_interface::HW_IF_VELOCITY + " expected.");
        }

        if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT) {
            throw std::runtime_error(
                "Joint " + joint.name + " has " + joint.state_interfaces[2].name +
                " as third state interface. " + hardware_interface::HW_IF_EFFORT + " expected.");
        }
    }
}

void RoverSystem::readDrivetrainSettings()
{
    drivetrain_settings_.motor_torque_constant =
        std::stof(info_.hardware_parameters["motor_torque_constant"]);
    drivetrain_settings_.gear_ratio =
        std::stof(info_.hardware_parameters["gear_ratio"]);
    drivetrain_settings_.gearbox_efficiency =
        std::stof(info_.hardware_parameters["gearbox_efficiency"]);
    drivetrain_settings_.encoder_resolution =
        std::stof(info_.hardware_parameters["encoder_resolution"]);
    drivetrain_settings_.max_rpm_motor_speed =
        std::stof(info_.hardware_parameters["max_rpm_motor_speed"]);
}

void RoverSystem::readDriverStatesUpdateFrequency()
{
    const float driver_states_update_frequency = 
        std::stof(info_.hardware_parameters["driver_states_update_frequency"]);
    driver_states_update_period_ = 
        rclcpp::Duration::from_seconds(1.0f / driver_states_update_frequency);
}

void RoverSystem::readDriverInitAndActivationAttempts()
{
    max_rover_driver_initialization_attempts_ =
        std::stoi(info_.hardware_parameters["max_rover_driver_initialization_attempts"]);
    max_rover_driver_activation_attempts_ =
        std::stoi(info_.hardware_parameters["max_rover_driver_activation_attempts"]);
}

void RoverSystem::configureRoverController()
{
    rover_driver_write_mtx_ = std::make_shared<std::mutex>();
    rover_controller_ = std::make_shared<RoverController>();

    rover_controller_->start();
    rover_controller_->eStopUserBtnTrigger(false);
    rover_controller_->eStopMotorDriverFaultTrigger(false);

    RCLCPP_INFO(logger_, "Successfully configured rover controller.");
}

void RoverSystem::configureRoverDriver()
{
    defineRoverDriver();

    if (!operationWithAttempts(
            std::bind(&RoverDriverInterface::initialize, rover_driver_),
            max_rover_driver_initialization_attempts_,
            std::bind(&RoverDriverInterface::deinitialize, rover_driver_))) {
        throw std::runtime_error("Rover drivers initialization failed.");
    }

    RCLCPP_INFO(logger_, "Successfully configured rover driver");
}

void RoverSystem::configureEStop()
{
    if (!rover_controller_ || !rover_driver_ || !rover_driver_write_mtx_) {
        throw std::runtime_error("Failed to configure E-Stop, make sure to setup entities first.");
    }

    e_stop_ = std::make_shared<EmergencyStop>(
        rover_controller_, rover_driver_, rover_driver_write_mtx_,
        std::bind(&RoverSystem::areVelocityCommandsNearZero, this));

    RCLCPP_INFO(logger_, "Successfully configured SW User E-Stop");
}

void RoverSystem::resetEStop()
{
    const auto lifecycle_state = this->get_lifecycle_state().id();

    if (lifecycle_state != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        throw std::runtime_error(
            "Can't reset sw user E-Stop when the hardware interface is not in ACTIVE state.");
    }

    e_stop_->resetEStop();
}

void RoverSystem::resetEStopLatch()
{
    const auto lifecycle_state = this->get_lifecycle_state().id();

    if (lifecycle_state != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        throw std::runtime_error(
            "Can't reset E-Stop Latch when the hardware interface is not in ACTIVE state.");
    }

    e_stop_->resetEStopLatch();
}

void RoverSystem::updateMotorsState(const rclcpp::Time & time)
{
    try {
        rover_driver_->updateMotorsState();
        updateHwStates(time);
    } catch (const std::runtime_error & e) {
        RCLCPP_ERROR_STREAM_THROTTLE(
            logger_, steady_clock_, 5000,
            "An exception occurred while updating motors states: " << e.what());
    }
}

void RoverSystem::updateDriverState()
{
    try {
        rover_driver_->updateDriversState();
    } catch (const std::runtime_error & e) {
        RCLCPP_ERROR_STREAM_THROTTLE(
            logger_, steady_clock_, 5000,
            "An exception occurred while updating drivers states: " << e.what());
    }
}

void RoverSystem::handleRoverDriverWriteOperation(std::function<void()> write_operation)
{
    try {
        {
            std::unique_lock<std::mutex> driver_write_lck(
                *rover_driver_write_mtx_, std::defer_lock);
            
            if (!driver_write_lck.try_lock()) {
                throw std::runtime_error(
                "Can't acquire mutex for writing commands");
            }
            
            write_operation();
        }
        
        // TODO: update error
    
    } catch (const std::runtime_error & e) {
        RCLCPP_WARN_STREAM(logger_, "An exception occurred while writing commands: " << e.what());
        
        // TODO: update error
    }
}

bool RoverSystem::areVelocityCommandsNearZero()
{
    for (const auto & cmd : hw_commands_velocities_) {
        if (std::abs(cmd) > std::numeric_limits<double>::epsilon()) {
            return false;
        }
    }

    return true;
}

}  // namespace rover_hardware_interface
