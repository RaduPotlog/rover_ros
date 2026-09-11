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

#include "rover_battery/application/monitor_battery_use_case.hpp"

#include <memory>
#include <stdexcept>
#include <utility>

#include "rover_battery/domain/battery_classifier.hpp"

namespace rover_battery::application
{

MonitorBatteryUseCase::MonitorBatteryUseCase(
    std::shared_ptr<domain::BatteryStatePublisherPort> publisher,
    domain::BatteryIdentity identity)
: publisher_(std::move(publisher))
, identity_(std::move(identity))
{
    if (!publisher_) {
        throw std::invalid_argument("MonitorBatteryUseCase requires a publisher");
    }
}

void MonitorBatteryUseCase::onFrame(const domain::BmsFrame & frame)
{
    last_cell_count_ = domain::validCellCount(frame.data);
    last_temp_sensor_count_ = domain::validTempSensorCount(frame.data);

    publisher_->publish(domain::buildBatteryReport(frame, identity_));
}

void MonitorBatteryUseCase::onDataTimeout()
{
    publisher_->publish(
        domain::staleBatteryReport(identity_, last_cell_count_, last_temp_sensor_count_));
}

}  // namespace rover_battery::application
