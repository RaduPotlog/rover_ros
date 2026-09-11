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

#ifndef ROVER_SAFETY_DOMAIN_BATTERY_SAFETY_POLICY_HPP_
#define ROVER_SAFETY_DOMAIN_BATTERY_SAFETY_POLICY_HPP_

#include <string>

namespace rover_safety::domain
{

/** Battery health as reported by the BMS. Mirrors sensor_msgs/BatteryState health values. */
enum class BatteryHealth
{
    Unknown,
    Good,
    Overheat,
    Dead,
    Overvoltage,
    UnspecFailure,
    Cold,
    WatchdogTimerExpire,
    SafetyTimerExpire,
};

/** What the safety layer must do about the battery. Ordered by severity. */
enum class SafetyVerdict : unsigned
{
    None = 0,
    TripEStop = 1,
    Shutdown = 2,
};

/** Battery temperature limits in degrees Celsius. Throws std::invalid_argument unless critical < fatal. */
class BatteryThresholds
{
public:
    BatteryThresholds(double critical_temp, double fatal_temp);

    double criticalTemp() const { return critical_temp_; }
    double fatalTemp() const { return fatal_temp_; }

private:
    double critical_temp_;
    double fatal_temp_;
};

struct BatterySafetyDecision
{
    SafetyVerdict verdict{SafetyVerdict::None};
    /** Human-readable cause; empty when verdict is None. */
    std::string reason;
};

/**
 * Decides the safety reaction to a battery reading:
 *  - watchdog expired, dead or overvoltage -> trip the e-stop;
 *  - overheat above the fatal limit -> shut the robot down;
 *  - overheat above the critical limit -> trip the e-stop;
 *  - anything else -> no action.
 */
BatterySafetyDecision evaluateBatterySafety(
    BatteryHealth health, double temperature, const BatteryThresholds & thresholds);

}  // namespace rover_safety::domain

#endif  // ROVER_SAFETY_DOMAIN_BATTERY_SAFETY_POLICY_HPP_
