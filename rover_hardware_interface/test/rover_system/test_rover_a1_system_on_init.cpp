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
//
// Integration test for RoverA1System driven through the real hardware_interface types
// (HardwareInfo / HardwareComponentInterfaceParams), the same way ros2_control's
// ResourceManager would populate them from a parsed URDF.
//
// Scope: on_init() and export_state_interfaces()/export_command_interfaces() only.
// RoverSystem::on_configure() unconditionally connects to a real Modbus TCP endpoint
// (configureRoverController()) and RoverA1System::defineRoverDriver() unconditionally talks to
// real Phidget USB hardware (configureRoverDriver()) - neither is mockable without a much larger
// refactor (injecting the safety-controller/driver construction), so on_configure()/on_activate()/
// read()/write() are intentionally NOT exercised here: doing so against a nonexistent Modbus
// endpoint would either hang (the URDF's default "retry forever" semantics for
// modbus_connection_retry_count=0) or make this test's pass/fail depend on the state of external
// hardware, which is exactly what this test suite must not do (see .claude/rules/testing.md).
//
// What IS exercised here never touches Modbus/Phidget: checkJointSize(), sortAndCheckJointNames(),
// checkInterfaces(), setInitialValues(), and the on_init()-time hardware_parameters readers
// (readDrivetrainSettings()/readDriverStatesUpdateFrequency()/
// readDriverInitAndActivationAttempts()/RoverA1System::readRoverControllerSettings()/
// readErrorFilterMaxErrorsCounts()).

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_component_interface_params.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include "rover_hardware_interface/rover_system/rover_a1_system.hpp"

namespace rover_hardware_interface
{
namespace
{

hardware_interface::ComponentInfo makeWheelJoint(const std::string & name)
{
    hardware_interface::ComponentInfo joint;
    joint.name = name;
    joint.type = "joint";

    hardware_interface::InterfaceInfo command_velocity;
    command_velocity.name = hardware_interface::HW_IF_VELOCITY;
    joint.command_interfaces.push_back(command_velocity);

    hardware_interface::InterfaceInfo state_position;
    state_position.name = hardware_interface::HW_IF_POSITION;
    hardware_interface::InterfaceInfo state_velocity;
    state_velocity.name = hardware_interface::HW_IF_VELOCITY;
    hardware_interface::InterfaceInfo state_effort;
    state_effort.name = hardware_interface::HW_IF_EFFORT;
    joint.state_interfaces.push_back(state_position);
    joint.state_interfaces.push_back(state_velocity);
    joint.state_interfaces.push_back(state_effort);

    return joint;
}

// Deliberately NOT in fl/fr/rl/rr order, to prove sortAndCheckJointNames() reorders exported
// interfaces to match RoverA1System's fixed joint_order_ regardless of URDF joint order.
hardware_interface::HardwareInfo buildValidHardwareInfo()
{
    hardware_interface::HardwareInfo info;
    info.name = "RoverA1System";
    info.type = "system";
    info.hardware_plugin_name = "rover_hardware_interface/RoverA1System";

    info.joints.push_back(makeWheelJoint("rr_wheel_base_to_rr_wheel_joint"));
    info.joints.push_back(makeWheelJoint("fl_wheel_base_to_fl_wheel_joint"));
    info.joints.push_back(makeWheelJoint("rl_wheel_base_to_rl_wheel_joint"));
    info.joints.push_back(makeWheelJoint("fr_wheel_base_to_fr_wheel_joint"));

    info.hardware_parameters = {
        // Modbus TCP endpoint for the safety controller. Never dialed in this test - on_init()
        // only parses these, it doesn't connect (see the file-level comment above).
        {"modbus_host", "127.0.0.1"},
        {"modbus_port", "502"},
        {"modbus_connection_retry_count", "1"},
        {"modbus_connection_retry_delay_ms", "10"},

        {"driver_states_update_frequency", "20"},
        {"max_rover_driver_initialization_attempts", "1"},
        {"max_rover_driver_activation_attempts", "1"},

        {"max_write_cmds_errors_count", "2"},
        {"max_read_motor_states_errors_count", "2"},
        {"max_read_driver_state_errors_count", "2"},

        {"driver_comm_timeout_ms", "300"},

        {"motor_torque_constant", "0.11"},
        {"max_rpm_motor_speed", "2800"},
        {"gear_ratio", "23.3"},
        {"gearbox_efficiency", "0.70"},
        {"raw_current_to_amps_scale", "0.1"},
        {"encoder_resolution", "1024"},
    };

    return info;
}

hardware_interface::HardwareComponentInterfaceParams makeParams(hardware_interface::HardwareInfo info)
{
    hardware_interface::HardwareComponentInterfaceParams params;
    params.hardware_info = std::move(info);
    return params;
}

}  // namespace

TEST(RoverA1SystemOnInit, SucceedsAndExportsInterfacesInFixedJointOrder)
{
    RoverA1System system;
    const auto params = makeParams(buildValidHardwareInfo());

    ASSERT_EQ(system.on_init(params), CallbackReturn::SUCCESS);

    const std::vector<std::string> expected_joint_order = {
        "fl_wheel_base_to_fl_wheel_joint", "fr_wheel_base_to_fr_wheel_joint",
        "rl_wheel_base_to_rl_wheel_joint", "rr_wheel_base_to_rr_wheel_joint"};

    auto state_interfaces = system.export_state_interfaces();
    ASSERT_EQ(state_interfaces.size(), expected_joint_order.size() * 3);

    for (std::size_t joint_idx = 0; joint_idx < expected_joint_order.size(); ++joint_idx) {
        const std::vector<std::string> expected_state_interfaces = {
            hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY,
            hardware_interface::HW_IF_EFFORT};

        for (std::size_t state_idx = 0; state_idx < expected_state_interfaces.size(); ++state_idx) {
            const auto & interface = state_interfaces[joint_idx * 3 + state_idx];
            EXPECT_EQ(interface.get_prefix_name(), expected_joint_order[joint_idx]);
            EXPECT_EQ(interface.get_interface_name(), expected_state_interfaces[state_idx]);
        }
    }

    auto command_interfaces = system.export_command_interfaces();
    ASSERT_EQ(command_interfaces.size(), expected_joint_order.size());

    for (std::size_t joint_idx = 0; joint_idx < expected_joint_order.size(); ++joint_idx) {
        const auto & interface = command_interfaces[joint_idx];
        EXPECT_EQ(interface.get_prefix_name(), expected_joint_order[joint_idx]);
        EXPECT_EQ(interface.get_interface_name(), hardware_interface::HW_IF_VELOCITY);
    }
}

TEST(RoverA1SystemOnInit, FailsWithWrongJointCount)
{
    RoverA1System system;
    auto info = buildValidHardwareInfo();
    info.joints.pop_back();  // Only 3 of the required 4 wheel joints.

    EXPECT_EQ(system.on_init(makeParams(info)), CallbackReturn::ERROR);
}

TEST(RoverA1SystemOnInit, FailsWithWrongCommandInterfaceType)
{
    RoverA1System system;
    auto info = buildValidHardwareInfo();
    info.joints[0].command_interfaces[0].name = hardware_interface::HW_IF_POSITION;

    EXPECT_EQ(system.on_init(makeParams(info)), CallbackReturn::ERROR);
}

TEST(RoverA1SystemOnInit, FailsWithMissingRequiredHardwareParameter)
{
    RoverA1System system;
    auto info = buildValidHardwareInfo();
    info.hardware_parameters.erase("modbus_host");

    EXPECT_EQ(system.on_init(makeParams(info)), CallbackReturn::ERROR);
}

TEST(RoverA1SystemOnInit, FailsWithEmptyModbusHost)
{
    RoverA1System system;
    auto info = buildValidHardwareInfo();
    info.hardware_parameters["modbus_host"] = "";

    EXPECT_EQ(system.on_init(makeParams(info)), CallbackReturn::ERROR);
}

}  // namespace rover_hardware_interface
