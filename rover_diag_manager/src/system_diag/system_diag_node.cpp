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

#include "rover_diag_manager/system_diag/system_diag_node.hpp"

#include <chrono>
#include <exception>

#include <cppuprofile/uprofile.h>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include "rover_msgs/msg/system_status.hpp"

#include "rover_diag_manager/system_diag/filesystem.hpp"
#include "rover_diag_manager/system_diag/types.hpp"
#include "rover_diag_manager/system_diag_params.hpp"

namespace rover_diag_manager
{

SystemDiagNode::SystemDiagNode(
    const std::string & node_name, 
    FilesystemInterface::SharedPtr filesystem,
    const rclcpp::NodeOptions & options)
: rclcpp::Node(node_name, options)
, filesystem_(filesystem)
, diagnostic_updater_(this)
{
    RCLCPP_INFO(this->get_logger(), "Initializing.");

    param_listener_ =
    std::make_shared<system_diag::ParamListener>(this->get_node_parameters_interface());
    params_ = param_listener_->get_params();

    system_status_publisher_ = this->create_publisher<rover_msgs::msg::SystemStatus>("system_status", 10);

    const auto timer_interval_ms = static_cast<long long>(1000.0 / params_.publish_frequency);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(timer_interval_ms),
        std::bind(&SystemDiagNode::timerCallback, this));

    diagnostic_updater_.setHardwareID("Rover PC");
    diagnostic_updater_.add("OS status", this, &SystemDiagNode::diagSystem);

    RCLCPP_INFO(this->get_logger(), "Initialized successfully.");
}

void SystemDiagNode::timerCallback()
{
    const auto status = getSystemStatus();
    auto message = systemStatusToRosMsg(status);

    system_status_publisher_->publish(message);
}

SystemStatus SystemDiagNode::getSystemStatus() const
{
    SystemStatus status;

    status.core_usages = getCoresUsages();
    status.cpu_mean_usage = getCpuMeanUsage(status.core_usages);
    status.cpu_temperature = getCpuTemperature();
    status.ram_usage = getRamUsage();
    status.disk_usage = getDiskUsage();

    return status;
}

std::vector<float> SystemDiagNode::getCoresUsages() const
{
    std::vector<float> loads = uprofile::getInstantCpuUsage();

    return loads;
}

float SystemDiagNode::getCpuMeanUsage(const std::vector<float> & usages) const
{
    if (usages.empty()) {
    return std::numeric_limits<float>::quiet_NaN();
    }

    std::for_each(usages.begin(), usages.end(), [](const float & usage) {
        if (usage < 0.0 || usage > 100.0) {
            throw std::invalid_argument{
                "At least one CPU core exceeds the valid usage range [0.0, 100.0]."};
        };
    });

    auto sum = std::accumulate(usages.begin(), usages.end(), 0.0);
    auto mean_usage = rover_diag_manager::setPrecision(sum / usages.size(), 2);

  return mean_usage;
}

float SystemDiagNode::getCpuTemperature() const
{
    float temperature = std::numeric_limits<float>::quiet_NaN();

    try {
        const auto temperature_str = filesystem_->readFile(kTemperatureInfoFilename);
        temperature = rover_diag_manager::setPrecision(std::stof(temperature_str) / 1000.0, 2);
    } catch (const std::exception & e) {
        RCLCPP_ERROR_STREAM(
            this->get_logger(), "An exception occurred while reading CPU temperature: " << e.what());
    }

    return temperature;
}

float SystemDiagNode::getRamUsage() const
{
    int total = 0, free = 0, available = 0;

    uprofile::getSystemMemory(total, available, free);

    const auto ram_usage = 0;//= rover_diag_manager::countPercentage(total - available, total);
  
    return ram_usage;
}

float SystemDiagNode::getDiskUsage() const
{
    float disk_usage = std::numeric_limits<float>::quiet_NaN();

    try {
        const auto capacity = filesystem_->getSpaceCapacity(kRootDirectory);
        const auto available = filesystem_->getSpaceAvailable(kRootDirectory);
        disk_usage = rover_diag_manager::countPercentage(capacity - available, capacity);
    } catch (const std::exception & e) {
        RCLCPP_ERROR_STREAM(
            this->get_logger(), "An exception occurred while reading disk usage: " << e.what());
    }

    return disk_usage;
}

rover_msgs::msg::SystemStatus SystemDiagNode::systemStatusToRosMsg(const SystemStatus & status)
{
    rover_msgs::msg::SystemStatus message;

    message.header.stamp = this->get_clock()->now();
    message.cpu_percent = status.core_usages;
    message.avg_load_percent = status.cpu_mean_usage;
    message.cpu_temp = status.cpu_temperature;
    message.ram_usage_percent = status.ram_usage;
    message.disc_usage_percent = status.disk_usage;

    return message;
}

void SystemDiagNode::diagSystem(diagnostic_updater::DiagnosticStatusWrapper & status)
{
    auto error_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string message = "System parameters are within acceptable limits.";

    params_ = param_listener_->get_params();
    auto system_status = getSystemStatus();

    status.add("CPU usage (%)", system_status.cpu_mean_usage);
    status.add("CPU temperature (°C)", system_status.cpu_temperature);
    status.add("RAM memory usage (%)", system_status.ram_usage);
    status.add("Disk memory usage (%)", system_status.disk_usage);

    std::unordered_map<double, diagnostic_msgs::msg::KeyValue> limits = {
        {params_.cpu_usage_warn_threshold, status.values[0]},
        {params_.cpu_temperature_warn_threshold, status.values[1]},
        {params_.ram_usage_warn_threshold, status.values[2]},
        {params_.disk_usage_warn_threshold, status.values[3]},
    };

    for (const auto & [limit, key_value] : limits) {
        if (std::isnan(std::stod(key_value.value))) {
            message = "Detected system parameter with unknown value.";
            error_level = diagnostic_updater::DiagnosticStatusWrapper::ERROR;
            break;
        } else if (std::stod(key_value.value) > limit) {
            message = "At least one system parameter is above the warning threshold.";
            error_level = diagnostic_updater::DiagnosticStatusWrapper::WARN;
        }
    }

    status.summary(error_level, message);
}

}  // namespace rover_diag_manager