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

#include <gtest/gtest.h>

#include <limits>
#include <string>

#include "rover_safety/domain/led_animation_policy.hpp"

using namespace rover_safety::domain;  // NOLINT

namespace
{

// config/led_safety.yaml
const LedBatteryThresholds kThresholds{0.1f, 0.4f, 0.05f};

LedAnimationDecision evaluate(
    const LedSafetyInputs & inputs, const LedBatteryThresholds & thresholds = kThresholds)
{
    return evaluateLedAnimation(inputs, thresholds);
}

LedSafetyInputs withPercent(double percent)
{
    LedSafetyInputs inputs;
    inputs.battery_status = PowerSupplyStatus::Discharging;
    inputs.battery_health = BatteryHealth::Good;
    inputs.battery_percent = percent;
    return inputs;
}

LedSafetyInputs withStatus(PowerSupplyStatus status, BatteryHealth health = BatteryHealth::Good)
{
    LedSafetyInputs inputs;
    inputs.battery_status = status;
    inputs.battery_health = health;
    inputs.battery_percent = 0.9;
    return inputs;
}

/** "critical", "low", "nominal", joined with '+', or "none". */
std::string levels(const LedAnimationDecision & decision)
{
    std::string text;
    const auto add = [&text](bool set, const char * name) {
        if (set) {
            text += (text.empty() ? "" : "+") + std::string(name);
        }
    };
    add(decision.critical_battery, "critical");
    add(decision.low_battery, "low");
    add(decision.nominal_battery, "nominal");
    return text.empty() ? "none" : text;
}

std::string levelsAt(double percent, const LedBatteryThresholds & thresholds = kThresholds)
{
    return levels(evaluate(withPercent(percent), thresholds));
}

}  // namespace

TEST(EvaluateLedAnimation, StateVerdict)
{
    const auto state = [](bool e_stop, bool dead_man) {
        LedSafetyInputs inputs;
        inputs.e_stop_pressed = e_stop;
        inputs.dead_man_held = dead_man;
        return evaluate(inputs).state;
    };

    EXPECT_EQ(state(false, false), LedStateVerdict::Ready);
    EXPECT_EQ(state(false, true), LedStateVerdict::ManualAction);
    EXPECT_EQ(state(true, false), LedStateVerdict::EStop);
    EXPECT_EQ(state(true, true), LedStateVerdict::EStop);
}

TEST(EvaluateLedAnimation, ErrorVerdict)
{
    const auto error = [](PowerSupplyStatus status, BatteryHealth health) {
        return evaluate(withStatus(status, health)).error;
    };

    EXPECT_EQ(error(PowerSupplyStatus::Unknown, BatteryHealth::Unknown), LedErrorVerdict::StatusUnknown);
    EXPECT_EQ(error(PowerSupplyStatus::Unknown, BatteryHealth::Good), LedErrorVerdict::StatusUnknown);
    EXPECT_EQ(error(PowerSupplyStatus::Unknown, BatteryHealth::Overheat), LedErrorVerdict::StatusUnknown);

    EXPECT_EQ(error(PowerSupplyStatus::Charging, BatteryHealth::Overheat), LedErrorVerdict::ChargingOverheat);
    EXPECT_EQ(error(PowerSupplyStatus::Charging, BatteryHealth::Good), LedErrorVerdict::None);
    EXPECT_EQ(error(PowerSupplyStatus::Charging, BatteryHealth::Unknown), LedErrorVerdict::None);

    // FULL counts as charging for the battery animations, not for the overheat error.
    EXPECT_EQ(error(PowerSupplyStatus::Full, BatteryHealth::Overheat), LedErrorVerdict::None);
    EXPECT_EQ(error(PowerSupplyStatus::Discharging, BatteryHealth::Overheat), LedErrorVerdict::None);
    EXPECT_EQ(error(PowerSupplyStatus::NotCharging, BatteryHealth::Overheat), LedErrorVerdict::None);
    EXPECT_EQ(error(PowerSupplyStatus(7), BatteryHealth::Overheat), LedErrorVerdict::None);
}

TEST(EvaluateLedAnimation, BatteryVerdict)
{
    const auto battery = [](unsigned status) {
        return evaluate(withStatus(static_cast<PowerSupplyStatus>(status))).battery;
    };

    EXPECT_EQ(battery(0), LedBatteryVerdict::None);
    EXPECT_EQ(battery(1), LedBatteryVerdict::Charging);
    EXPECT_EQ(battery(2), LedBatteryVerdict::Discharging);
    EXPECT_EQ(battery(3), LedBatteryVerdict::Discharging);
    EXPECT_EQ(battery(4), LedBatteryVerdict::Charging);
    EXPECT_EQ(battery(5), LedBatteryVerdict::None);
    EXPECT_EQ(battery(255), LedBatteryVerdict::None);
}

TEST(EvaluateLedAnimation, BatteryLevelsAtThresholds)
{
    EXPECT_EQ(levelsAt(0.0), "critical");
    EXPECT_EQ(levelsAt(0.05), "critical");
    EXPECT_EQ(levelsAt(0.0999), "critical");

    EXPECT_EQ(levelsAt(0.1), "low");
    EXPECT_EQ(levelsAt(0.2), "low");
    EXPECT_EQ(levelsAt(0.3999), "low");

    EXPECT_EQ(levelsAt(0.4), "nominal");
    EXPECT_EQ(levelsAt(0.9), "nominal");
    EXPECT_EQ(levelsAt(1.0), "nominal");
    EXPECT_EQ(levelsAt(std::numeric_limits<double>::infinity()), "nominal");
    EXPECT_EQ(levelsAt(-std::numeric_limits<double>::infinity()), "critical");

    // The percentage is compared as a float, like the blackboard entry the tree used to read.
    EXPECT_EQ(levelsAt(0.1 - 1e-12), "low");
    EXPECT_EQ(levelsAt(0.4 - 1e-12), "nominal");
}

TEST(EvaluateLedAnimation, NanPercentSelectsNoLevel)
{
    const auto decision = evaluate(withPercent(std::numeric_limits<double>::quiet_NaN()));

    EXPECT_EQ(levels(decision), "none");
    EXPECT_FALSE(decision.battery_full);
    EXPECT_NE(decision.battery_percent_round.find("nan"), std::string::npos)
        << decision.battery_percent_round;
}

TEST(EvaluateLedAnimation, MisorderedThresholdsAreAccepted)
{
    const LedBatteryThresholds misordered{0.5f, 0.3f, 0.05f};

    EXPECT_EQ(levelsAt(0.4, misordered), "critical+nominal");
    EXPECT_EQ(levelsAt(0.2, misordered), "critical");
    EXPECT_EQ(levelsAt(0.6, misordered), "nominal");
}

TEST(RoundChargingPercent, MatchesTheNodeFormula)
{
    EXPECT_EQ(roundChargingPercent(0.0, 0.05f), "0.000000");
    EXPECT_EQ(roundChargingPercent(0.5, 0.05f), "0.500000");
    EXPECT_EQ(roundChargingPercent(0.52f, 0.05f), "0.500000");
    EXPECT_EQ(roundChargingPercent(0.53f, 0.05f), "0.550000");
    EXPECT_EQ(roundChargingPercent(0.974f, 0.05f), "0.950000");
    EXPECT_EQ(roundChargingPercent(0.976f, 0.05f), "1.000000");
    EXPECT_EQ(roundChargingPercent(1.0, 0.05f), "1.000000");

    EXPECT_EQ(roundChargingPercent(0.94f, 0.1f), "0.900000");
    EXPECT_EQ(roundChargingPercent(0.96f, 0.1f), "1.000000");
}

TEST(EvaluateLedAnimation, FullOnlyWhenRoundedTextReadsOne)
{
    LedSafetyInputs inputs = withStatus(PowerSupplyStatus::Charging);

    inputs.battery_percent = 0.976f;
    auto decision = evaluate(inputs);
    EXPECT_TRUE(decision.battery_full);
    EXPECT_EQ(decision.battery_percent_round, "1.000000");

    inputs.battery_percent = 0.974f;
    decision = evaluate(inputs);
    EXPECT_FALSE(decision.battery_full);
    EXPECT_EQ(decision.battery_percent_round, "0.950000");

    inputs.battery_percent = 0.96f;
    EXPECT_TRUE(evaluate(inputs, {0.1f, 0.4f, 0.1f}).battery_full);
}

TEST(EvaluateLedAnimation, ChannelsAreIndependent)
{
    LedSafetyInputs inputs;
    inputs.e_stop_pressed = true;
    inputs.dead_man_held = true;
    inputs.battery_status = PowerSupplyStatus::Charging;
    inputs.battery_health = BatteryHealth::Overheat;
    inputs.battery_percent = 0.05;

    const auto decision = evaluate(inputs);
    EXPECT_EQ(decision.state, LedStateVerdict::EStop);
    EXPECT_EQ(decision.error, LedErrorVerdict::ChargingOverheat);
    EXPECT_EQ(decision.battery, LedBatteryVerdict::Charging);
    EXPECT_EQ(levels(decision), "critical");
}
