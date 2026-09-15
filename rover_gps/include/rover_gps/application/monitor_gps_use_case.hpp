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

#ifndef ROVER_GPS_APPLICATION_MONITOR_GPS_USE_CASE_HPP_
#define ROVER_GPS_APPLICATION_MONITOR_GPS_USE_CASE_HPP_

#include <memory>

#include "rover_gps/domain/gnss_fix.hpp"
#include "rover_gps/domain/gps_health_evaluator.hpp"
#include "rover_gps/domain/ports/gps_health_publisher_port.hpp"

namespace rover_gps::application
{

/** @brief Feeds GNSS fixes to the health evaluator and publishes its verdict periodically. */
class MonitorGpsUseCase
{
public:
    MonitorGpsUseCase(
        std::shared_ptr<domain::GpsHealthPublisherPort> publisher,
        domain::GpsHealthThresholds thresholds);

    void onFix(const domain::GnssFix & fix);

    /** @brief Evaluates at `now_s` (also catches a stream that stopped) and publishes. */
    void onTick(double now_s);

private:
    std::shared_ptr<domain::GpsHealthPublisherPort> publisher_;
    domain::GpsHealthEvaluator evaluator_;
};

}  // namespace rover_gps::application

#endif  // ROVER_GPS_APPLICATION_MONITOR_GPS_USE_CASE_HPP_
