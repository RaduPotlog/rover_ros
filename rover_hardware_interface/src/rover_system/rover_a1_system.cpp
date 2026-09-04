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

#include "rover_hardware_interface/rover_system/rover_a1_system.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/logging.hpp"
#include "diagnostic_updater/diagnostic_status_wrapper.hpp"

#include "rover_hardware_interface/rover_driver/rover_a1_driver.hpp"
#include "rover_hardware_interface/rover_safety_controller/rover_safety_controller_e_stop_io.hpp"
#include "rover_hardware_interface/rover_safety_controller/rover_safety_controller_gpio_adapter.hpp"

namespace rover_hardware_interface
{

RoverA1System::RoverA1System()
: RoverSystem(joints_)
{

}

void RoverA1System::defineRoverDriver()
{
    rover_driver_ = std::make_shared<RoverA1Driver>(drivetrain_settings_);
}

void RoverA1System::readRoverControllerSettings()
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

void RoverA1System::defineRoverController()
{
    // Scoped local: only needed long enough to construct the two port adapters below; never
    // stored as a member so nothing on the RT read()/write() path can reach the concrete
    // Modbus-backed type directly (see rover_system.hpp's RoverGpioPort/EmergencyStopInterface
    // member comments).
    auto rover_safety_controller_impl = std::make_shared<RoverSafetyController>(
        modbus_settings_.host, modbus_settings_.port,
        modbus_settings_.connection_retry_count,
        std::chrono::milliseconds(modbus_settings_.connection_retry_delay_ms));

    rover_controller_ = std::make_shared<RoverSafetyControllerGpioAdapter>(rover_safety_controller_impl);

    auto e_stop_io = std::make_shared<RoverSafetyControllerEStopIo>(rover_safety_controller_impl);
    // Forming a pointer to a protected base-class member requires naming it through the derived
    // class (this translation unit's own class), not the base - see [class.protected].
    e_stop_ = std::make_shared<EmergencyStop>(
        e_stop_io, std::bind(&RoverA1System::areVelocityCommandsNearZero, this));
}

void RoverA1System::updateHwStates(const rclcpp::Time & time)
{
    (void)time;

    const auto rear_left_data = rover_driver_->getData(DriverNames::REAR_LEFT);
    const auto rear_right_data = rover_driver_->getData(DriverNames::REAR_RIGHT);

    const auto front_left_data = rover_driver_->getData(DriverNames::FRONT_LEFT);
    const auto front_right_data = rover_driver_->getData(DriverNames::FRONT_RIGHT);

    const auto rear_left = rear_left_data.getMotorState(MotorNames::DEFAULT);
    const auto rear_right = rear_right_data.getMotorState(MotorNames::DEFAULT);
    const auto front_left = front_left_data.getMotorState(MotorNames::DEFAULT);
    const auto front_right = front_right_data.getMotorState(MotorNames::DEFAULT);

    // joint_order_ (rover_a1_system.hpp) is {"fl", "fr", "rl", "rr"}, so index 0=front_left,
    // 1=front_right, 2=rear_left, 3=rear_right.
    hw_states_positions_[0] = front_left.getPosition();
    hw_states_positions_[1] = front_right.getPosition();
    hw_states_positions_[2] = rear_left.getPosition();
    hw_states_positions_[3] = rear_right.getPosition();

    hw_states_velocities_[0] = front_left.getVelocity();
    hw_states_velocities_[1] = front_right.getVelocity();
    hw_states_velocities_[2] = rear_left.getVelocity();
    hw_states_velocities_[3] = rear_right.getVelocity();

    hw_states_efforts_[0] = front_left.getTorque();
    hw_states_efforts_[1] = front_right.getTorque();
    hw_states_efforts_[2] = rear_left.getTorque();
    hw_states_efforts_[3] = rear_right.getTorque();
}

void RoverA1System::updateDriverStateMsg()
{
    const auto driver_front_left_data = rover_driver_->getData(DriverNames::FRONT_LEFT);
    const auto driver_front_right_data = rover_driver_->getData(DriverNames::FRONT_RIGHT);

    const auto driver_rear_left_data = rover_driver_->getData(DriverNames::REAR_LEFT);
    const auto driver_rear_right_data = rover_driver_->getData(DriverNames::REAR_RIGHT);

    system_ros_interface_->updateMsgDriversStates(DriverNames::REAR_LEFT,
        driver_rear_left_data.getDriverState());
    system_ros_interface_->updateMsgErrorFlags(DriverNames::REAR_LEFT,
        driver_rear_left_data);

    system_ros_interface_->updateMsgDriversStates(DriverNames::REAR_RIGHT,
        driver_rear_right_data.getDriverState());
    system_ros_interface_->updateMsgErrorFlags(DriverNames::REAR_RIGHT,
        driver_rear_right_data);

    system_ros_interface_->updateMsgDriversStates(DriverNames::FRONT_LEFT,
        driver_front_left_data.getDriverState());
    system_ros_interface_->updateMsgErrorFlags(DriverNames::FRONT_LEFT,
        driver_front_left_data);

    system_ros_interface_->updateMsgDriversStates(DriverNames::FRONT_RIGHT,
        driver_front_right_data.getDriverState());
    system_ros_interface_->updateMsgErrorFlags(DriverNames::FRONT_RIGHT,
        driver_front_right_data);

    system_ros_interface_->updateMsgError(rover_error_filter_->isError());
}

void RoverA1System::diagnoseErrors(diagnostic_updater::DiagnosticStatusWrapper & status)
{
    unsigned char level{diagnostic_updater::DiagnosticStatusWrapper::OK};
    std::string message{"No error detected."};

    const auto front_left_driver_data = rover_driver_->getData(DriverNames::FRONT_LEFT);

    if (front_left_driver_data.isError()) {
        level = diagnostic_updater::DiagnosticStatusWrapper::ERROR;
        message = "Error detected.";

        RoverA1System::addKeyValueIfTrue(
            status, front_left_driver_data.getErrorMap(), "Front left driver error: ");
    }

    const auto front_right_driver_data = rover_driver_->getData(DriverNames::FRONT_RIGHT);

    if (front_right_driver_data.isError()) {
        level = diagnostic_updater::DiagnosticStatusWrapper::ERROR;
        message = "Error detected.";

        RoverA1System::addKeyValueIfTrue(
            status, front_right_driver_data.getErrorMap(), "Front right driver error: ");
    }

    const auto rear_left_driver_data = rover_driver_->getData(DriverNames::REAR_LEFT);

    if (rear_left_driver_data.isError()) {
        level = diagnostic_updater::DiagnosticStatusWrapper::ERROR;
        message = "Error detected.";

        RoverA1System::addKeyValueIfTrue(
            status, rear_left_driver_data.getErrorMap(), "Rear left driver error: ");
    }

    const auto rear_right_driver_data = rover_driver_->getData(DriverNames::REAR_RIGHT);

    if (rear_right_driver_data.isError()) {
        level = diagnostic_updater::DiagnosticStatusWrapper::ERROR;
        message = "Error detected.";

        RoverA1System::addKeyValueIfTrue(
            status, rear_right_driver_data.getErrorMap(), "Rear right driver error: ");
    }

    if (rover_error_filter_->isError()) {
        level = diagnostic_updater::DiagnosticStatusWrapper::ERROR;
        message = "Error detected.";

        RoverA1System::addKeyValueIfTrue(
            status, rover_error_filter_->getErrorMap(), "Error filter - ");
    }

    status.summary(level, message);
}

void RoverA1System::diagnoseStatus(diagnostic_updater::DiagnosticStatusWrapper & status)
{
    unsigned char level{diagnostic_updater::DiagnosticStatusWrapper::OK};
    std::string message{"Rover A1 system status monitoring."};

    status.add("Communication error", rover_driver_->isCommunicationError());

    const auto front_left_driver_state = rover_driver_->getData(DriverNames::FRONT_LEFT).getDriverState();
    const auto front_right_driver_state = rover_driver_->getData(DriverNames::FRONT_RIGHT).getDriverState();
    const auto rear_left_driver_state = rover_driver_->getData(DriverNames::REAR_LEFT).getDriverState();
    const auto rear_right_driver_state = rover_driver_->getData(DriverNames::REAR_RIGHT).getDriverState();

    auto driver_states_with_names = {
        std::make_pair(std::string("Front Left"), front_left_driver_state),
        std::make_pair(std::string("Front Right"), front_right_driver_state),
        std::make_pair(std::string("Rear Left"), rear_left_driver_state),
        std::make_pair(std::string("Rear Right"), rear_right_driver_state)};

    for (const auto & [driver_name, driver_state] : driver_states_with_names) {
        status.add(driver_name + " driver current (A)", driver_state.getDriverCurrent());
        status.add(driver_name + " driver temperature (\u00B0C)", driver_state.getTemperature());
    }

    status.summary(level, message);
}

void RoverA1System::getSpeedCmd(std::vector<float> & speed_cmd) const
{
    speed_cmd[0] = static_cast<float>(hw_commands_velocities_[0]);
    speed_cmd[1] = static_cast<float>(hw_commands_velocities_[1]);
    speed_cmd[2] = static_cast<float>(hw_commands_velocities_[2]);
    speed_cmd[3] = static_cast<float>(hw_commands_velocities_[3]);
}

}  // namespace rover_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    rover_hardware_interface::RoverA1System, hardware_interface::SystemInterface)
