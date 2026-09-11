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

#include "rover_diag_manager/infrastructure/linux_system_metrics_source.hpp"

#include <cppuprofile/uprofile.h>

#include <exception>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rover_diag_manager/domain/percentage.hpp"

namespace rover_diag_manager::infrastructure
{

namespace
{
constexpr int kLogThrottleMs = 10000;
constexpr unsigned int kTemperatureDecimals = 2;
}  // namespace

LinuxSystemMetricsSource::LinuxSystemMetricsSource(
    FilesystemInterface::SharedPtr filesystem, rclcpp::Logger logger)
: filesystem_(std::move(filesystem))
, logger_(std::move(logger))
{
    if (!filesystem_) {
        throw std::invalid_argument("LinuxSystemMetricsSource requires a filesystem");
    }
}

domain::SystemSample LinuxSystemMetricsSource::sample()
{
    domain::SystemSample sample;

    try {
        sample.core_usages = uprofile::getInstantCpuUsage();
    } catch (const std::exception & e) {
        RCLCPP_ERROR_STREAM_THROTTLE(logger_, throttle_clock_, kLogThrottleMs,
            "An exception occurred while reading CPU usage: " << e.what());
    } catch (...) {
        RCLCPP_ERROR_THROTTLE(logger_, throttle_clock_, kLogThrottleMs,
            "An unknown exception occurred while reading CPU usage.");
    }
    sample.cpu_mean_usage = domain::meanUsage(sample.core_usages);

    sample.cpu_temperature = readCpuTemperature();
    sample.ram_usage = readRamUsage();
    sample.disk_usage = readDiskUsage();

    return sample;
}

std::optional<float> LinuxSystemMetricsSource::readCpuTemperature()
{
    try {
        // The thermal zone reports millidegrees Celsius.
        const float millidegrees = std::stof(filesystem_->readFile(kTemperatureInfoFilename));
        return domain::roundTo(millidegrees / 1000.0f, kTemperatureDecimals);
    } catch (const std::exception & e) {
        RCLCPP_ERROR_STREAM_THROTTLE(logger_, throttle_clock_, kLogThrottleMs,
            "An exception occurred while reading CPU temperature: " << e.what());
    } catch (...) {
        RCLCPP_ERROR_THROTTLE(logger_, throttle_clock_, kLogThrottleMs,
            "An unknown exception occurred while reading CPU temperature.");
    }
    return std::nullopt;
}

std::optional<float> LinuxSystemMetricsSource::readRamUsage()
{
    try {
        // [kB], parsed from /proc/meminfo.
        int total = 0;
        int available = 0;
        int free = 0;
        uprofile::getSystemMemory(total, available, free);

        const auto usage = domain::percentageOf(total - available, total);
        if (!usage) {
            RCLCPP_ERROR_THROTTLE(logger_, throttle_clock_, kLogThrottleMs,
                "Total system memory reported as zero.");
        }
        return usage;
    } catch (const std::exception & e) {
        RCLCPP_ERROR_STREAM_THROTTLE(logger_, throttle_clock_, kLogThrottleMs,
            "An exception occurred while reading RAM usage: " << e.what());
    } catch (...) {
        RCLCPP_ERROR_THROTTLE(logger_, throttle_clock_, kLogThrottleMs,
            "An unknown exception occurred while reading RAM usage.");
    }
    return std::nullopt;
}

std::optional<float> LinuxSystemMetricsSource::readDiskUsage()
{
    try {
        const auto capacity = static_cast<double>(filesystem_->getSpaceCapacity(kRootDirectory));
        const auto available = static_cast<double>(filesystem_->getSpaceAvailable(kRootDirectory));

        const auto usage = domain::percentageOf(capacity - available, capacity);
        if (!usage) {
            RCLCPP_ERROR_THROTTLE(logger_, throttle_clock_, kLogThrottleMs,
                "Root filesystem capacity reported as zero.");
        }
        return usage;
    } catch (const std::exception & e) {
        RCLCPP_ERROR_STREAM_THROTTLE(logger_, throttle_clock_, kLogThrottleMs,
            "An exception occurred while reading disk usage: " << e.what());
    } catch (...) {
        RCLCPP_ERROR_THROTTLE(logger_, throttle_clock_, kLogThrottleMs,
            "An unknown exception occurred while reading disk usage.");
    }
    return std::nullopt;
}

}  // namespace rover_diag_manager::infrastructure
