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

#ifndef ROVER_DIAG_MANAGER_INFRASTRUCTURE_LINUX_SYSTEM_METRICS_SOURCE_HPP_
#define ROVER_DIAG_MANAGER_INFRASTRUCTURE_LINUX_SYSTEM_METRICS_SOURCE_HPP_

#include <optional>

#include "rclcpp/rclcpp.hpp"

#include "rover_diag_manager/domain/ports/system_metrics_source_port.hpp"
#include "rover_diag_manager/infrastructure/filesystem.hpp"

namespace rover_diag_manager::infrastructure
{

/**
 * @brief Reads CPU and RAM usage via cppuprofile, CPU temperature from the thermal zone and
 *        root-filesystem usage via FilesystemInterface.
 * @note  sample() blocks for ~100 ms: cppuprofile's instant CPU usage polls twice.
 */
class LinuxSystemMetricsSource : public domain::SystemMetricsSourcePort
{
public:
    LinuxSystemMetricsSource(FilesystemInterface::SharedPtr filesystem, rclcpp::Logger logger);

    domain::SystemSample sample() override;

    static constexpr char kTemperatureInfoFilename[] = "/sys/class/thermal/thermal_zone0/temp";
    static constexpr char kRootDirectory[] = "/";

private:
    std::optional<float> readCpuTemperature();
    std::optional<float> readRamUsage();
    std::optional<float> readDiskUsage();

    FilesystemInterface::SharedPtr filesystem_;
    rclcpp::Logger logger_;
    rclcpp::Clock throttle_clock_{RCL_STEADY_TIME};
};

}  // namespace rover_diag_manager::infrastructure

#endif  // ROVER_DIAG_MANAGER_INFRASTRUCTURE_LINUX_SYSTEM_METRICS_SOURCE_HPP_
