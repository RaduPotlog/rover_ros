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
// Integration test for PhidgetImuSensor driven through the real hardware_interface types
// (HardwareInfo / HardwareComponentInterfaceParams), the same way ros2_control's
// ResourceManager would populate them from a parsed URDF (see
// rover_description/urdf/common/imu.urdf.xacro's <ros2_control type="sensor"> block, which wires
// this plugin into the real, non-simulated bringup path - confirmed by tracing
// rover_a1_macro.urdf.xacro's unconditional xacro:imu.imu invocation and
// rover_controller.launch.py's imu_broadcaster_spawner).
//
// Scope: on_init() and export_state_interfaces() only, mirroring
// test/rover_system/test_rover_a1_system_on_init.cpp. PhidgetImuSensor::on_activate()
// unconditionally opens a real Phidget Spatial USB connection and blocks on calibration
// (calibrate()), so on_configure()/on_activate()/read() are intentionally NOT exercised here -
// doing so without real IMU hardware attached would either fail unpredictably or time out (see
// .claude/rules/testing.md on not depending on external hardware state).
//
// What IS exercised here never touches the Phidget SDK: checkSensorName(), checkStatesSize(),
// checkInterfaces(), and setInitialValues() - all pure validation/allocation against the
// hand-built HardwareInfo, matching PhidgetImuSensor::on_init()'s try block exactly.

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_component_interface_params.hpp>

#include "rover_hardware_interface/rover_sensors/phidget_imu_sensor.hpp"

namespace rover_hardware_interface
{
namespace
{

// Mirrors PhidgetImuSensor's protected kImuInterfacesNames (see phidget_imu_sensor.hpp) - not
// reachable from here since it's protected, so re-listed in the exact order checkInterfaces()
// requires (it compares by position via std::equal, not by set membership).
const std::vector<std::string> kExpectedImuInterfaceNames = {
    "orientation.x",         "orientation.y",         "orientation.z",      "orientation.w",
    "angular_velocity.x",    "angular_velocity.y",    "angular_velocity.z", "linear_acceleration.x",
    "linear_acceleration.y", "linear_acceleration.z",
};

hardware_interface::ComponentInfo makeImuComponentInfo(const std::string & name)
{
    hardware_interface::ComponentInfo sensor;
    sensor.name = name;
    sensor.type = "sensor";

    for (const auto & interface_name : kExpectedImuInterfaceNames) {
        hardware_interface::InterfaceInfo state_interface;
        state_interface.name = interface_name;
        sensor.state_interfaces.push_back(state_interface);
    }

    return sensor;
}

hardware_interface::HardwareInfo buildValidHardwareInfo()
{
    hardware_interface::HardwareInfo info;
    info.name = "imu";
    info.type = "sensor";
    info.hardware_plugin_name = "rover_hardware_interface/PhidgetImuSensor";

    info.sensors.push_back(makeImuComponentInfo("imu"));

    return info;
}

hardware_interface::HardwareComponentInterfaceParams makeParams(hardware_interface::HardwareInfo info)
{
    hardware_interface::HardwareComponentInterfaceParams params;
    params.hardware_info = std::move(info);
    return params;
}

}  // namespace

TEST(PhidgetImuSensorOnInit, SucceedsAndExportsStateInterfacesInDeclaredOrder)
{
    PhidgetImuSensor sensor;
    const auto params = makeParams(buildValidHardwareInfo());

    ASSERT_EQ(sensor.on_init(params), CallbackReturn::SUCCESS);

    auto state_interfaces = sensor.export_state_interfaces();
    ASSERT_EQ(state_interfaces.size(), kExpectedImuInterfaceNames.size());

    for (std::size_t i = 0; i < kExpectedImuInterfaceNames.size(); ++i) {
        EXPECT_EQ(state_interfaces[i].get_prefix_name(), "imu");
        EXPECT_EQ(state_interfaces[i].get_interface_name(), kExpectedImuInterfaceNames[i]);
    }
}

TEST(PhidgetImuSensorOnInit, FailsWithNoSensorDefined)
{
    PhidgetImuSensor sensor;
    auto info = buildValidHardwareInfo();
    info.sensors.clear();

    EXPECT_EQ(sensor.on_init(makeParams(info)), CallbackReturn::ERROR);
}

TEST(PhidgetImuSensorOnInit, FailsWithWrongStateInterfaceCount)
{
    PhidgetImuSensor sensor;
    auto info = buildValidHardwareInfo();
    info.sensors.at(0).state_interfaces.pop_back();  // Only 9 of the required 10 interfaces.

    EXPECT_EQ(sensor.on_init(makeParams(info)), CallbackReturn::ERROR);
}

TEST(PhidgetImuSensorOnInit, FailsWithMismatchedInterfaceName)
{
    PhidgetImuSensor sensor;
    auto info = buildValidHardwareInfo();
    // checkInterfaces() compares positionally, so a renamed (not just reordered) entry trips it.
    info.sensors.at(0).state_interfaces.at(0).name = "not_a_real_interface";

    EXPECT_EQ(sensor.on_init(makeParams(info)), CallbackReturn::ERROR);
}

}  // namespace rover_hardware_interface
