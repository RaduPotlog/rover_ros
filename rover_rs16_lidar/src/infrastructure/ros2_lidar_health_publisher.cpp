// Copyright 2026 Mechatronics Academy
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

#include "rover_rs16_lidar/infrastructure/ros2_lidar_health_publisher.hpp"

#include <memory>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"

namespace rover_rs16_lidar::infrastructure
{

using DiagnosticStatusMsg = diagnostic_msgs::msg::DiagnosticStatus;

unsigned char toDiagnosticLevel(domain::HealthLevel level)
{
    switch (level) {
        case domain::HealthLevel::Ok:
            return DiagnosticStatusMsg::OK;
        case domain::HealthLevel::Warn:
            return DiagnosticStatusMsg::WARN;
        case domain::HealthLevel::Error:
            return DiagnosticStatusMsg::ERROR;
        default:
            return DiagnosticStatusMsg::STALE;
    }
}

Ros2LidarHealthPublisher::Ros2LidarHealthPublisher(
    const std::shared_ptr<diagnostic_updater::Updater> & diagnostic_updater)
{
    diagnostic_updater->add("Lidar status", this, &Ros2LidarHealthPublisher::diagnose);
}

void Ros2LidarHealthPublisher::publish(const domain::LidarHealthReport & report)
{
    report_ = report;
}

void Ros2LidarHealthPublisher::diagnose(diagnostic_updater::DiagnosticStatusWrapper & status)
{
    if (!report_) {
        status.summary(diagnostic_updater::DiagnosticStatusWrapper::STALE, "No lidar data yet.");
        return;
    }

    status.add("Cloud count", report_->cloud_count);

    if (report_->has_data) {
        status.add("Points", report_->point_count);
        status.addf("Rate (Hz)", "%.2f", report_->rate_hz);
        status.addf("Age (s)", "%.1f", report_->age_s);
    }

    status.summary(toDiagnosticLevel(report_->level), report_->message);
}

}  // namespace rover_rs16_lidar::infrastructure
