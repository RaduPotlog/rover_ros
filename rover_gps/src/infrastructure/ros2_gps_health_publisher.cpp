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

#include "rover_gps/infrastructure/ros2_gps_health_publisher.hpp"

#include <cmath>
#include <memory>

#include "rover_gps/infrastructure/gps_msg_conversions.hpp"

namespace rover_gps::infrastructure
{

Ros2GpsHealthPublisher::Ros2GpsHealthPublisher(
    const std::shared_ptr<diagnostic_updater::Updater> & diagnostic_updater)
{
    diagnostic_updater->add("GPS fix", this, &Ros2GpsHealthPublisher::diagnose);
}

void Ros2GpsHealthPublisher::publish(const domain::GpsHealthReport & report)
{
    report_ = report;
}

void Ros2GpsHealthPublisher::diagnose(diagnostic_updater::DiagnosticStatusWrapper & status)
{
    if (!report_) {
        status.summary(diagnostic_updater::DiagnosticStatusWrapper::STALE, "No GPS data yet.");
        return;
    }

    status.add("Fix count", report_->fix_count);

    if (report_->has_data) {
        const auto & fix = report_->last_fix;
        status.add("Fix status", fixStatusText(fix.status));
        status.addf("Latitude (deg)", "%.7f", fix.latitude_deg);
        status.addf("Longitude (deg)", "%.7f", fix.longitude_deg);
        status.addf("Altitude (m)", "%.2f", fix.altitude_m);
        if (std::isnan(fix.horizontal_std_m)) {
            status.add("Horizontal std (m)", "unknown");
        } else {
            status.addf("Horizontal std (m)", "%.2f", fix.horizontal_std_m);
        }
        status.addf("Rate (Hz)", "%.2f", report_->rate_hz);
        status.addf("Age (s)", "%.1f", report_->age_s);
    }

    status.summary(toDiagnosticLevel(report_->level), report_->message);
}

}  // namespace rover_gps::infrastructure
