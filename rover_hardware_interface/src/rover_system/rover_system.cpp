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
#include <exception>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <diagnostic_updater/diagnostic_status_wrapper.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp/logging.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>

#include "rover_hardware_interface/rover_controller/rover_controller_e_stop_io.hpp"
#include "rover_hardware_interface/rover_controller/rover_controller_gpio_adapter.hpp"
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
        readModbusSettings();
        readErrorFilterMaxErrorsCounts();
    } catch (const std::exception & e) {
        // Widened from std::invalid_argument: std::stof/std::stoi can also throw
        // std::out_of_range, and a missing key (operator[] replaced with .at() below) throws
        // std::out_of_range too - both must fail on_init() cleanly instead of escaping uncaught.
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
    } catch (const std::exception & e) {
        // Widened from std::runtime_error: MB::ModbusException (thrown by RoverModbus during
        // e.g. coil initialization in configureRoverController()) derives directly from
        // std::exception, not std::runtime_error, and must not escape on_configure() uncaught.
        RCLCPP_ERROR_STREAM(logger_, "Failed to configure Rover System. Error: " << e.what());
        // configureRoverController() may have already started rover_controller_ (and its
        // background poll thread) before a later step in this try block failed - tear it down
        // rather than leaving it running past a failed configure.
        teardownRoverComponents();
        return CallbackReturn::ERROR;
    }

    std::fill(hw_commands_velocities_.begin(), hw_commands_velocities_.end(), 0.0);
    std::fill(hw_states_positions_.begin(), hw_states_positions_.end(), 0.0);
    std::fill(hw_states_velocities_.begin(), hw_states_velocities_.end(), 0.0);
    std::fill(hw_states_efforts_.begin(), hw_states_efforts_.end(), 0.0);

    // Force read() to re-seed next_driver_state_update_time_ from the first `time` it
    // receives after this (re)configure, rather than comparing against a value seeded under
    // a previous activation - see the member's declaration for why the clock type must match.
    driver_state_update_time_initialized_ = false;

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

    system_ros_interface_->addDiagnosticTask(
    std::string("system errors"), this, &RoverSystem::diagnoseErrors);

    system_ros_interface_->addDiagnosticTask(
    std::string("system status"), this, &RoverSystem::diagnoseStatus);

    const auto & gpio_state = rover_controller_->queryControlInterfaceIOStates();
    system_ros_interface_->updateMsgGpioStates(gpio_state);
    system_ros_interface_->publishGpioStateMsg();

    return CallbackReturn::SUCCESS;
}

CallbackReturn RoverSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
    teardownRoverComponents();

    return CallbackReturn::SUCCESS;
}

CallbackReturn RoverSystem::on_activate(const rclcpp_lifecycle::State &)
{
    // Actually engaging the motors (sending a zero-velocity command to each driver and waiting
    // for them to arm) belongs here, not in on_configure() — a merely-configured component
    // should not yet be commanding hardware.
    if (!operationWithAttempts(
            std::bind(&RoverDriverInterface::activate, rover_driver_),
            max_rover_driver_activation_attempts_, []() {}, std::chrono::milliseconds(0),
            [this](const std::string & message) { RCLCPP_WARN_STREAM(logger_, message); })) {
        RCLCPP_ERROR_STREAM(logger_, "Failed to activate Rover System: Couldn't activate RoverDriver in "
                                     << max_rover_driver_activation_attempts_ << " attempts.");
        return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
}

CallbackReturn RoverSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
    return CallbackReturn::SUCCESS;
}

CallbackReturn RoverSystem::on_shutdown(const rclcpp_lifecycle::State &)
{
    // Valid from Unconfigured too (e.g. controller_manager shutting down a component whose
    // on_configure() was never reached), where teardownRoverComponents()'s null check on
    // rover_driver_ is what keeps this from null-derefing.
    teardownRoverComponents();

    return CallbackReturn::SUCCESS;
}

CallbackReturn RoverSystem::on_error(const rclcpp_lifecycle::State &)
{
    if (e_stop_) {
        try {
            e_stop_->setEStop();
        } catch (const std::exception & e) {
            // Widened from std::runtime_error: MB::ModbusException derives directly from
            // std::exception, not std::runtime_error, and must not escape on_error() - the
            // error-handling callback itself - uncaught.
            RCLCPP_ERROR_STREAM(logger_, "Handling error failed: " << e.what());
            return CallbackReturn::ERROR;
        }
    }

    if (system_ros_interface_) {
        system_ros_interface_->broadcastOnDiagnosticTasks(
            diagnostic_msgs::msg::DiagnosticStatus::ERROR,
            "An error has occurred during a node state transition.");
    }

    // Delegate the common reset to teardownRoverComponents() so this path and on_cleanup()/
    // on_shutdown() can't drift apart as owned resources are added/changed.
    teardownRoverComponents();

    return CallbackReturn::SUCCESS;
}

void RoverSystem::teardownRoverComponents()
{
    rover_controller_.reset();
    rover_controller_impl_.reset();

    if (rover_driver_) {
        rover_driver_->deinitialize();
        rover_driver_.reset();
    }

    e_stop_.reset();
    system_ros_interface_.reset();
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
    updateCommunicationStatus();

    // Seed on the first cycle (or the first cycle after a (re)configure) from `time` itself,
    // rather than a fixed clock type: `time` is whatever clock the resource manager is using
    // (e.g. RCL_ROS_TIME), and rclcpp::Time comparisons throw std::runtime_error when the two
    // operands were constructed with different clock types.
    if (!driver_state_update_time_initialized_) {
        next_driver_state_update_time_ = time;
        driver_state_update_time_initialized_ = true;
    }

    if (time >= next_driver_state_update_time_) {
        updateDriverState();
        updateFlagErrors();
        updateDriverStateMsg();
        system_ros_interface_->publishRobotDriverState();

        const auto & gpio_state = rover_controller_->queryControlInterfaceIOStates();
        system_ros_interface_->updateMsgGpioStates(gpio_state);
        system_ros_interface_->publishGpioStateMsg();

        next_driver_state_update_time_ = time + driver_states_update_period_;
    }

    updateEStopState();

    return return_type::OK;
}

return_type RoverSystem::write(const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
{
    // get_lifecycle_state() is documented as not real-time safe and not for use in the
    // control loop; get_lifecycle_id() is the cached, RT-safe equivalent (see
    // hardware_interface::HardwareComponentInterface).
    const auto lifecycle_state = this->get_lifecycle_id();

    if (lifecycle_state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        if (e_stop_active_) {
            // E-Stop (user-triggered or latched) is active: do not command motion in software,
            // regardless of what the hardware relay is doing.
            return return_type::OK;
        }

        handleRoverDriverWriteOperation([this] {
            getSpeedCmd(speed_cmd_buffer_);
            rover_driver_->sendSpeedCmd(speed_cmd_buffer_);
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

    speed_cmd_buffer_.resize(joint_size_, 0.0f);
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
        std::stof(info_.hardware_parameters.at("motor_torque_constant"));
    drivetrain_settings_.gear_ratio =
        std::stof(info_.hardware_parameters.at("gear_ratio"));
    drivetrain_settings_.gearbox_efficiency =
        std::stof(info_.hardware_parameters.at("gearbox_efficiency"));
    drivetrain_settings_.encoder_resolution =
        std::stof(info_.hardware_parameters.at("encoder_resolution"));
    drivetrain_settings_.max_rpm_motor_speed =
        std::stof(info_.hardware_parameters.at("max_rpm_motor_speed"));
    drivetrain_settings_.driver_comm_timeout_ms =
        static_cast<unsigned>(std::stoi(info_.hardware_parameters.at("driver_comm_timeout_ms")));
}

void RoverSystem::readDriverStatesUpdateFrequency()
{
    const float driver_states_update_frequency =
        std::stof(info_.hardware_parameters.at("driver_states_update_frequency"));
    driver_states_update_period_ =
        rclcpp::Duration::from_seconds(1.0f / driver_states_update_frequency);
}

void RoverSystem::readDriverInitAndActivationAttempts()
{
    max_rover_driver_initialization_attempts_ =
        std::stoi(info_.hardware_parameters.at("max_rover_driver_initialization_attempts"));
    max_rover_driver_activation_attempts_ =
        std::stoi(info_.hardware_parameters.at("max_rover_driver_activation_attempts"));
}

void RoverSystem::readModbusSettings()
{
    modbus_settings_.host = info_.hardware_parameters.at("modbus_host");

    if (modbus_settings_.host.empty()) {
        throw std::invalid_argument("Missing or empty 'modbus_host' hardware parameter.");
    }

    modbus_settings_.port = std::stoi(info_.hardware_parameters.at("modbus_port"));

    modbus_settings_.connection_retry_count =
        static_cast<unsigned>(std::stoi(info_.hardware_parameters.at("modbus_connection_retry_count")));
    modbus_settings_.connection_retry_delay_ms =
        static_cast<unsigned>(std::stoi(info_.hardware_parameters.at("modbus_connection_retry_delay_ms")));
}

void RoverSystem::readErrorFilterMaxErrorsCounts()
{
    const unsigned max_write_cmds_errors_count = static_cast<unsigned>(
        std::stoi(info_.hardware_parameters.at("max_write_cmds_errors_count")));
    const unsigned max_read_motor_states_errors_count = static_cast<unsigned>(
        std::stoi(info_.hardware_parameters.at("max_read_motor_states_errors_count")));
    const unsigned max_read_driver_state_errors_count = static_cast<unsigned>(
        std::stoi(info_.hardware_parameters.at("max_read_driver_state_errors_count")));

    // Not URDF-configurable, mirroring the reference Roboteq implementation's fault-flag
    // category: a single occurrence escalates immediately, with no debounce.
    constexpr unsigned kMaxFaultFlagErrorsCount = 1;

    rover_error_filter_ = std::make_unique<RoverErrorFilter>(
        max_write_cmds_errors_count, max_read_motor_states_errors_count,
        max_read_driver_state_errors_count, kMaxFaultFlagErrorsCount);
}

void RoverSystem::configureRoverController()
{
    rover_driver_write_mtx_ = std::make_shared<std::mutex>();
    rover_controller_impl_ = std::make_shared<RoverController>(
        modbus_settings_.host, modbus_settings_.port,
        modbus_settings_.connection_retry_count,
        std::chrono::milliseconds(modbus_settings_.connection_retry_delay_ms));

    rover_controller_ = std::make_shared<RoverControllerGpioAdapter>(rover_controller_impl_);

    rover_controller_->start();
    // TODO(mechatronics-academy): Check if e-stop interface can be used
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
            std::bind(&RoverDriverInterface::deinitialize, rover_driver_), std::chrono::milliseconds(0),
            [this](const std::string & message) { RCLCPP_WARN_STREAM(logger_, message); })) {
        throw std::runtime_error("Rover drivers initialization failed.");
    }

    RCLCPP_INFO(logger_, "Successfully configured rover driver");
}

void RoverSystem::configureEStop()
{
    if (!rover_controller_impl_) {
        throw std::runtime_error("Failed to configure E-Stop, make sure to setup entities first.");
    }

    auto e_stop_io = std::make_shared<RoverControllerEStopIo>(rover_controller_impl_);

    e_stop_ = std::make_shared<EmergencyStop>(
        e_stop_io, std::bind(&RoverSystem::areVelocityCommandsNearZero, this));

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

    // Resetting the driver-fault latch is the operator's explicit signal that the underlying
    // fault has been addressed - clear all accumulated per-category error-filter state too, not
    // just the fault-flag category.
    rover_error_filter_->setClearErrorsFlag();
}

void RoverSystem::updateMotorsState(const rclcpp::Time & time)
{
    try {
        rover_driver_->updateMotorsState();
        updateHwStates(time);
        updateMotorsStateDataTimedOut();
    } catch (const std::runtime_error & e) {
        RCLCPP_ERROR_STREAM_THROTTLE(
            logger_, steady_clock_, 5000,
            "An exception occurred while updating motors states: " << e.what());
        rover_error_filter_->updateError(ErrorsFilterIds::READ_MOTOR_STATES, true);
    }
}

void RoverSystem::updateMotorsStateDataTimedOut()
{
    if (rover_driver_->isMotorStatesDataTimedOut()) {
        rover_error_filter_->updateError(ErrorsFilterIds::READ_MOTOR_STATES, true);
    } else {
        rover_error_filter_->updateError(ErrorsFilterIds::READ_MOTOR_STATES, false);
    }
}

void RoverSystem::updateDriverState()
{
    try {
        rover_driver_->updateDriversState();
        updateDriverStateDataTimedOut();
    } catch (const std::runtime_error & e) {
        RCLCPP_ERROR_STREAM_THROTTLE(
            logger_, steady_clock_, 5000,
            "An exception occurred while updating drivers states: " << e.what());
        rover_error_filter_->updateError(ErrorsFilterIds::READ_DRIVER_STATE, true);
    }
}

void RoverSystem::updateDriverStateDataTimedOut()
{
    if (rover_driver_->isDriverStateDataTimedOut()) {
        rover_error_filter_->updateError(ErrorsFilterIds::READ_DRIVER_STATE, true);
    } else {
        rover_error_filter_->updateError(ErrorsFilterIds::READ_DRIVER_STATE, false);
    }
}

void RoverSystem::updateFlagErrors()
{
    if (rover_driver_->isFlagError()) {
        rover_error_filter_->updateError(ErrorsFilterIds::FAULT_FLAG, true);
        handleRoverDriverWriteOperation([this] { rover_driver_->attemptErrorFlagReset(); });
    } else {
        rover_error_filter_->updateError(ErrorsFilterIds::FAULT_FLAG, false);
    }
}

void RoverSystem::updateCommunicationStatus()
{
    try {
        rover_driver_->updateCommunicationStatus();
    } catch (const std::runtime_error & e) {
        RCLCPP_ERROR_STREAM_THROTTLE(
            logger_, steady_clock_, 5000,
            "An exception occurred while updating communication status: " << e.what());
    }
}

void RoverSystem::updateEStopState()
{
    if (!e_stop_) {
        e_stop_active_ = true;
        return;
    }

    const bool user_e_stop_triggered = e_stop_->readEStopState();
    const bool e_stop_latched = e_stop_->readEStopLatchState();

    e_stop_active_ = user_e_stop_triggered || e_stop_latched;
}

void RoverSystem::handleRoverDriverWriteOperation(std::function<void()> write_operation)
{
    std::unique_lock<std::mutex> driver_write_lck(*rover_driver_write_mtx_, std::defer_lock);

    if (!driver_write_lck.try_lock()) {
        // Lock contention (e.g. a concurrent E-Stop reset/service call) is expected and
        // recoverable — skip this write cycle rather than pay exception cost on every
        // contended RT tick.
        RCLCPP_WARN_STREAM_THROTTLE(
            logger_, steady_clock_, 5000,
            "Couldn't acquire mutex for writing commands; skipping this write cycle.");
        rover_error_filter_->updateError(ErrorsFilterIds::WRITE_CMDS, true);
        return;
    }

    try {
        write_operation();
        rover_error_filter_->updateError(ErrorsFilterIds::WRITE_CMDS, false);
    } catch (const std::exception & e) {
        // Widened from std::runtime_error: write_operation() ultimately calls down into
        // std::unordered_map::at() (RoverA1Driver::sendSpeedCmd()), which throws
        // std::out_of_range - a sibling of std::runtime_error, not caught by it - on a lookup
        // miss. write() must never let an exception escape, so this boundary catches
        // std::exception broadly, matching the same widening already applied in on_init().
        RCLCPP_WARN_STREAM_THROTTLE(
            logger_, steady_clock_, 5000,
            "An exception occurred while writing commands: " << e.what());
        rover_error_filter_->updateError(ErrorsFilterIds::WRITE_CMDS, true);
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
