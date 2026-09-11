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

#ifndef ROVER_BATTERY_APPLICATION_MONITOR_BATTERY_USE_CASE_HPP_
#define ROVER_BATTERY_APPLICATION_MONITOR_BATTERY_USE_CASE_HPP_

#include <cstddef>
#include <memory>

#include "rover_battery/domain/battery_report.hpp"
#include "rover_battery/domain/bms_frame.hpp"
#include "rover_battery/domain/ports/battery_state_publisher_port.hpp"

namespace rover_battery::application
{

/**
 * @brief Turns BMS frames (or their absence) into published battery reports.
 * @details Remembers only the last reported cell/sensor counts, so the stale report keeps
 *          the same array shape as the last valid one.
 */
class MonitorBatteryUseCase
{
public:
    MonitorBatteryUseCase(
        std::shared_ptr<domain::BatteryStatePublisherPort> publisher,
        domain::BatteryIdentity identity);

    void onFrame(const domain::BmsFrame & frame);

    void onDataTimeout();

private:
    std::shared_ptr<domain::BatteryStatePublisherPort> publisher_;
    domain::BatteryIdentity identity_;

    std::size_t last_cell_count_{0};
    std::size_t last_temp_sensor_count_{0};
};

}  // namespace rover_battery::application

#endif  // ROVER_BATTERY_APPLICATION_MONITOR_BATTERY_USE_CASE_HPP_
