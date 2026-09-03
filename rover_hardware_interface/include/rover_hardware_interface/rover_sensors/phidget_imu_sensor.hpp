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

#ifndef ROVER_HARDWARE_INTERFACE_PHIDGET_IMU_SENSOR_PHIDGET_IMU_SENSOR_HPP_
#define ROVER_HARDWARE_INTERFACE_PHIDGET_IMU_SENSOR_PHIDGET_IMU_SENSOR_HPP_

#include <array>
#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_set>
#include <vector>

#include "imu_filter_madgwick/imu_filter.h"
#include "imu_filter_madgwick/world_frame.h"
#include "phidgets_api/spatial.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/hardware_component_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3.hpp"

#include "rover_hardware_interface/domain/imu_calibration.hpp"
#include "rover_hardware_interface/phidgets_spatial_parameters.hpp"

namespace rover_hardware_interface
{

using return_type = hardware_interface::return_type;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using StateInterface = hardware_interface::StateInterface;
using CommandInterface = hardware_interface::CommandInterface;

class PhidgetImuSensor : public hardware_interface::SensorInterface
{

public:

    static constexpr size_t kImuInterfacesSize = 10;
    static constexpr double KImuMagneticFieldUnknownValue = 1e300;
    static constexpr float G = 9.80665;

    CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams & params) override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

    std::vector<StateInterface> export_state_interfaces() override;

    return_type read(const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */) override;

protected:

    void checkSensorName() const;
    void checkStatesSize() const;

    void setInitialValues();
    void checkInterfaces() const;
    void readObligatoryParams();
    void readCompassParams();
    void readMadgwickFilterParams();
    void checkMadgwickFilterWorldFrameParam();
    bool isParamDefined(const std::string & param_name) const;
    bool areParamsDefined(const std::unordered_set<std::string> & params_names) const;

    // Fall back to `default_value` (matching phidgets_spatial_parameters.yaml's declared
    // default_value for the same key) when the URDF hardware parameter is not present, instead of
    // throwing via info_.hardware_parameters.at(). Logs at INFO so a missing-but-defaulted param
    // is still visible, unlike a silent fallback.
    int readIntParamOrDefault(const std::string & param_name, const int default_value) const;
    double readDoubleParamOrDefault(const std::string & param_name, const double default_value) const;
    bool readBoolParamOrDefault(const std::string & param_name, const bool default_value) const;

    void configureCompassParams();
    void configureHeating();
    void configureMadgwickFilter();

    void spatialDataCallback(
        const double acceleration[3],
        const double angular_rate[3],
        const double magnetic_field[3],
        const double timestamp);
    void spatialAttachCallback();
    void spatialDetachCallback();

    geometry_msgs::msg::Vector3 parseMagnitude(const double magnetic_field[3]);
    geometry_msgs::msg::Vector3 parseGyration(const double angular_rate[3]);
    geometry_msgs::msg::Vector3 parseAcceleration(const double acceleration[3]);

    void initializeMadgwickAlgorithm(const geometry_msgs::msg::Vector3 & mag_compensated,
                                     const geometry_msgs::msg::Vector3 & lin_acc,
                                     const rclcpp::Time & timestamp);

    void restartMadgwickAlgorithm();

    bool isIMUCalibrated(const geometry_msgs::msg::Vector3 & mag_compensated);

    void updateMadgwickAlgorithm(const geometry_msgs::msg::Vector3 & ang_vel, const geometry_msgs::msg::Vector3 & lin_acc,
                                 const geometry_msgs::msg::Vector3 & mag_compensated, const double dt);
    void updateMadgwickAlgorithmIMU(const geometry_msgs::msg::Vector3 & ang_vel, const geometry_msgs::msg::Vector3 & lin_acc, const double dt);
    void updateAccelerationAndGyrationStateValues(const geometry_msgs::msg::Vector3 & ang_vel, const geometry_msgs::msg::Vector3 & lin_acc);
    void updateAllStatesValues(const geometry_msgs::msg::Vector3 & ang_vel, const geometry_msgs::msg::Vector3 & lin_acc);

    void setStateValuesToNans();

    void calibrate();

    // Copies imu_sensor_state_staging_ into imu_sensor_state_ (the buffer whose addresses are
    // handed out by export_state_interfaces()). Called once per read() cycle on the RT thread;
    // non-blocking, so it never stalls the RT loop waiting on the Phidget SDK callback thread.
    void updateExportedStateValues();

    // The buffer backing the exported StateInterfaces. Only ever written by
    // updateExportedStateValues() (RT thread), so reads of it by the hardware_interface
    // framework (same thread, after read()) are race-free.
    std::vector<double> imu_sensor_state_;

    // Written by the Phidget SDK's callback thread (spatialDataCallback / spatialDetachCallback);
    // guarded by imu_sensor_state_mtx_. Never read directly by the RT thread.
    std::vector<double> imu_sensor_state_staging_;
    std::mutex imu_sensor_state_mtx_;

    rclcpp::Logger logger_{rclcpp::get_logger("PhidgetImuSensor")};
    rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

    inline static const std::array<std::string, kImuInterfacesSize> kImuInterfacesNames = {
        "orientation.x",         "orientation.y",         "orientation.z",      "orientation.w",
        "angular_velocity.x",    "angular_velocity.y",    "angular_velocity.z", "linear_acceleration.x",
        "linear_acceleration.y", "linear_acceleration.z",
    };

    enum ImuMeasurements {
        orientation_x,
        orientation_y,
        orientation_z,
        orientation_w,
        angular_velocity_x,
        angular_velocity_y,
        angular_velocity_z,
        linear_acceleration_x,
        linear_acceleration_y,
        linear_acceleration_z
    };

    phidgets_spatial::Params params_;
    std::unique_ptr<phidgets::Spatial> spatial_;

    std::unique_ptr<ImuFilter> filter_;
    // Defaults to ENU to match checkMadgwickFilterWorldFrameParam()'s documented fallback when
    // the 'world_frame' hardware parameter is missing/invalid - previously left uninitialized.
    WorldFrame::WorldFrame world_frame_ = WorldFrame::ENU;
    std::atomic_bool imu_connected_{false};

    ImuCalibrationGate imu_calibration_gate_;
    std::mutex calibration_mutex_;
    std::condition_variable calibration_cv_;

    bool algorithm_initialized_ = false;
    rclcpp::Time last_spatial_data_timestamp_;
};

}  // namespace rover_hardware_interface

#endif  // ROVER_HARDWARE_INTERFACE_PHIDGET_IMU_SENSOR_PHIDGET_IMU_SENSOR_HPP_
