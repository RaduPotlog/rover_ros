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

#ifndef ROVER_DIAG_MANAGER_SYSTEM_DIAG_SYSTEM_DIAG_NODE_HPP_
#define ROVER_DIAG_MANAGER_SYSTEM_DIAG_SYSTEM_DIAG_NODE_HPP_

#include <string>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include "rover_diag_manager/system_diag/types.hpp"
#include "rover_diag_manager/system_diag/filesystem.hpp"
#include "rover_diag_manager/system_diag_params.hpp"

#include "rover_msgs/msg/system_status.hpp"

namespace rover_diag_manager
{

template <typename T>
T setPrecision(T value, unsigned int precision)
{
    static_assert(
        std::is_floating_point<T>::value,
        "SetPrecision method can only be used with floating point types.");

    std::ostringstream out;
    out << std::fixed << std::setprecision(precision) << value;

    return std::stof(out.str());
}

template <typename T>
float countPercentage(const T & value, const T & total)
{
    if (total == 0) {
        throw std::invalid_argument("Total must not be zero.");
    }

    auto percentage = static_cast<float>(value) / static_cast<float>(total) * 100.0;
    
    return setPrecision(percentage, 2);
}

class SystemDiagNode : public rclcpp::Node
{

public:

    SystemDiagNode(
        const std::string & node_name, FilesystemInterface::SharedPtr filesystem,
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:

    SystemStatus getSystemStatus() const;

    std::vector<float> getCoresUsages() const;
    float getCpuMeanUsage(const std::vector<float> & usages) const;
    float getCpuTemperature() const;
    float getRamUsage() const;
    float getDiskUsage() const;

    rover_msgs::msg::SystemStatus systemStatusToRosMsg(const SystemStatus & status);

private:

    void timerCallback();
    
    void diagSystem(diagnostic_updater::DiagnosticStatusWrapper & status);

    FilesystemInterface::SharedPtr filesystem_;
    diagnostic_updater::Updater diagnostic_updater_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<rover_msgs::msg::SystemStatus>::SharedPtr system_status_publisher_;

    system_diag::Params params_;
    std::shared_ptr<system_diag::ParamListener> param_listener_;

    static constexpr char kTemperatureInfoFilename[] = "/sys/class/thermal/thermal_zone0/temp";
    static constexpr char kRootDirectory[] = "/";

};

}  // namespace rover_diag_manager

#endif  // ROVER_DIAG_MANAGER_SYSTEM_DIAG_SYSTEM_DIAG_NODE_HPP_