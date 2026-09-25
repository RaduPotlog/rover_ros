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

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <iterator>
#include <limits>
#include <ostream>
#include <string>
#include <thread>
#include <vector>

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>

#include "rover_msgs/msg/led_animation.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

#include "rover_safety/domain/led_animation_policy.hpp"
#include "rover_safety/infrastructure/battery_state_conversion.hpp"
#include "rover_safety/plugins/decorator/tick_after_timeout_node.hpp"

using namespace std::chrono_literals;
using BatteryStateMsg = sensor_msgs::msg::BatteryState;
using LedAnimationMsg = rover_msgs::msg::LedAnimation;
namespace domain = rover_safety::domain;
namespace infrastructure = rover_safety::infrastructure;

namespace
{

struct LedAnimationCall
{
    unsigned id;
    std::string param;
    bool repeating;

    bool operator==(const LedAnimationCall & other) const
    {
        return id == other.id && param == other.param && repeating == other.repeating;
    }
};

void PrintTo(const LedAnimationCall & call, std::ostream * os)
{
    *os << "{id " << call.id << ", param \"" << call.param << "\", repeating "
        << (call.repeating ? "true" : "false") << "}";
}

using Calls = std::vector<LedAnimationCall>;

/**
 * Stands in for CallSetLedAnimationService: same ID and ports, reads id, param and repeating like
 * its on_tick() and records them instead of calling led/set_animation. It answers at once, like an
 * LED server that replies success within bt_loop_duration.
 */
class RecordingSetLedAnimation : public BT::SyncActionNode
{
public:
    RecordingSetLedAnimation(const std::string & name, const BT::NodeConfig & config, Calls * calls)
    : BT::SyncActionNode(name, config)
    , calls_(calls)
    {
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("service_name", "please_set_service_name_in_BT_Node"),
            BT::InputPort<std::chrono::milliseconds>("server_timeout"),
            BT::InputPort<unsigned>("id", "Animation ID to trigger."),
            BT::InputPort<std::string>("param", "Optional animation parameter."),
            BT::InputPort<bool>("repeating", "Specifies whether the animation should repeated continuously."),
        };
    }

    BT::NodeStatus tick() override
    {
        LedAnimationCall call{};
        if (!getInput<unsigned>("id", call.id) || !getInput<std::string>("param", call.param) ||
            !getInput<bool>("repeating", call.repeating)) {
            return BT::NodeStatus::FAILURE;
        }
        calls_->push_back(call);
        return BT::NodeStatus::SUCCESS;
    }

private:
    Calls * calls_;
};

// config/led_safety.yaml.
const domain::LedBatteryThresholds kShippedThresholds{0.1f, 0.4f, 0.05f};
// battery.anim_period.low is 30 s; shortened so the LOW_BATTERY repeat fits in a test.
constexpr float kLowBatteryAnimPeriod = 0.3f;

LedAnimationCall call(unsigned id, const std::string & param, bool repeating)
{
    return {id, param, repeating};
}

const LedAnimationCall kReady = call(LedAnimationMsg::READY, "", true);
const LedAnimationCall kManualAction = call(LedAnimationMsg::MANUAL_ACTION, "", true);
const LedAnimationCall kEStop = call(LedAnimationMsg::E_STOP, "", true);
const LedAnimationCall kError = call(LedAnimationMsg::ERROR, "", true);
const LedAnimationCall kNoError = call(LedAnimationMsg::NO_ERROR, "", true);
const LedAnimationCall kChargerInserted = call(LedAnimationMsg::CHARGER_INSERTED, "", false);
const LedAnimationCall kBatteryNominal = call(LedAnimationMsg::BATTERY_NOMINAL, "", false);

const std::vector<unsigned> kStateIds{
    LedAnimationMsg::READY, LedAnimationMsg::MANUAL_ACTION, LedAnimationMsg::E_STOP};
const std::vector<unsigned> kErrorIds{LedAnimationMsg::ERROR, LedAnimationMsg::NO_ERROR};
const std::vector<unsigned> kBatteryIds{
    LedAnimationMsg::CHARGER_INSERTED, LedAnimationMsg::CHARGING_BATTERY,
    LedAnimationMsg::BATTERY_CHARGED, LedAnimationMsg::LOW_BATTERY,
    LedAnimationMsg::CRITICAL_BATTERY, LedAnimationMsg::BATTERY_NOMINAL};

/**
 * Ticks the shipped RoverLedSafety tree (RoverLedSafetyBT.btproj) the way rover_led_safety_node
 * does: blackboard filled like createLedInitialBlackboard(), inputs written like its subscription
 * callbacks (raw entries and led_* verdicts), ticked at 10 Hz. Only led/set_animation is replaced.
 */
class LedSafetyTreeTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        // Plugins first, then the project, like registerBehaviorTree().
        factory_.registerNodeType<rover_safety::SafetyBtTickAfterTimeout>("SafetyBtTickAfterTimeout");
        factory_.registerNodeType<RecordingSetLedAnimation>("CallSetLedAnimationService", &calls_);
        factory_.registerBehaviorTreeFromFile(ROVER_SAFETY_LED_BT_PROJECT);
    }

    /** A fresh tree, as after the node's configure. */
    void createTree(const domain::LedBatteryThresholds & thresholds = kShippedThresholds)
    {
        thresholds_ = thresholds;
        inputs_ = {};
        blackboard_ = BT::Blackboard::create();

        // LedSafetyNode::createLedInitialBlackboard().
        blackboard_->set<std::string>("charging_anim_percent", "");
        blackboard_->set<int>("current_anim_id", -1);
        blackboard_->set<int>("current_battery_anim_id", -1);
        blackboard_->set<int>("current_error_anim_id", -1);
        blackboard_->set<bool>("drive_state", false);
        blackboard_->set<float>("CRITICAL_BATTERY_THRESHOLD_PERCENT", thresholds.critical_percent);
        blackboard_->set<float>("LOW_BATTERY_ANIM_PERIOD", kLowBatteryAnimPeriod);
        blackboard_->set<float>("LOW_BATTERY_THRESHOLD_PERCENT", thresholds.low_percent);
        blackboard_->set<unsigned>("E_STOP_ANIM_ID", LedAnimationMsg::E_STOP);
        blackboard_->set<unsigned>("READY_ANIM_ID", LedAnimationMsg::READY);
        blackboard_->set<unsigned>("ERROR_ANIM_ID", LedAnimationMsg::ERROR);
        blackboard_->set<unsigned>("NO_ERROR_ANIM_ID", LedAnimationMsg::NO_ERROR);
        blackboard_->set<unsigned>("MANUAL_ACTION_ANIM_ID", LedAnimationMsg::MANUAL_ACTION);
        blackboard_->set<unsigned>("LOW_BATTERY_ANIM_ID", LedAnimationMsg::LOW_BATTERY);
        blackboard_->set<unsigned>("CRITICAL_BATTERY_ANIM_ID", LedAnimationMsg::CRITICAL_BATTERY);
        blackboard_->set<unsigned>("CHARGING_BATTERY_ANIM_ID", LedAnimationMsg::CHARGING_BATTERY);
        blackboard_->set<unsigned>("BATTERY_CHARGED_ANIM_ID", LedAnimationMsg::BATTERY_CHARGED);
        blackboard_->set<unsigned>("CHARGER_INSERTED_ANIM_ID", LedAnimationMsg::CHARGER_INSERTED);
        blackboard_->set<unsigned>("BATTERY_NOMINAL_ANIM_ID", LedAnimationMsg::BATTERY_NOMINAL);
        blackboard_->set<unsigned>(
            "POWER_SUPPLY_STATUS_UNKNOWN", BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN);
        blackboard_->set<unsigned>(
            "POWER_SUPPLY_STATUS_CHARGING", BatteryStateMsg::POWER_SUPPLY_STATUS_CHARGING);
        blackboard_->set<unsigned>(
            "POWER_SUPPLY_STATUS_DISCHARGING", BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING);
        blackboard_->set<unsigned>(
            "POWER_SUPPLY_STATUS_NOT_CHARGING", BatteryStateMsg::POWER_SUPPLY_STATUS_NOT_CHARGING);
        blackboard_->set<unsigned>(
            "POWER_SUPPLY_STATUS_FULL", BatteryStateMsg::POWER_SUPPLY_STATUS_FULL);
        blackboard_->set<unsigned>(
            "POWER_SUPPLY_HEALTH_OVERHEAT", BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERHEAT);
        blackboard_->set<unsigned>("LED_STATE_READY", unsigned(domain::LedStateVerdict::Ready));
        blackboard_->set<unsigned>(
            "LED_STATE_MANUAL_ACTION", unsigned(domain::LedStateVerdict::ManualAction));
        blackboard_->set<unsigned>("LED_STATE_E_STOP", unsigned(domain::LedStateVerdict::EStop));
        blackboard_->set<unsigned>("LED_ERROR_NONE", unsigned(domain::LedErrorVerdict::None));
        blackboard_->set<unsigned>(
            "LED_ERROR_CHARGING_OVERHEAT", unsigned(domain::LedErrorVerdict::ChargingOverheat));
        blackboard_->set<unsigned>(
            "LED_ERROR_STATUS_UNKNOWN", unsigned(domain::LedErrorVerdict::StatusUnknown));
        blackboard_->set<unsigned>("LED_BATTERY_NONE", unsigned(domain::LedBatteryVerdict::None));
        blackboard_->set<unsigned>(
            "LED_BATTERY_CHARGING", unsigned(domain::LedBatteryVerdict::Charging));
        blackboard_->set<unsigned>(
            "LED_BATTERY_DISCHARGING", unsigned(domain::LedBatteryVerdict::Discharging));
        // Read by the real CallSetLedAnimationService (nav2 BtServiceNode) only.
        blackboard_->set<std::chrono::milliseconds>("server_timeout", 5000ms);
        blackboard_->set<std::chrono::milliseconds>("bt_loop_duration", 10ms);
        blackboard_->set<std::chrono::milliseconds>("wait_for_service_timeout", 3000ms);

        tree_ = factory_.createTree("RoverLedSafety", blackboard_);
        calls_.clear();
    }

    /** LedSafetyNode::batteryCallback(). */
    void publishBattery(std::uint8_t status, std::uint8_t health, float percentage)
    {
        blackboard_->set<unsigned>("battery_status", status);
        blackboard_->set<unsigned>("battery_health", health);

        if (status != BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN &&
            health != BatteryStateMsg::POWER_SUPPLY_HEALTH_UNKNOWN) {
            inputs_.battery_percent = percentage;
        }
        inputs_.battery_status = infrastructure::toPowerSupplyStatus(status);
        inputs_.battery_health = infrastructure::toBatteryHealth(health);

        const auto decision = updateLedVerdicts();
        blackboard_->set<float>("battery_percent", inputs_.battery_percent);
        blackboard_->set<std::string>("battery_percent_round", decision.battery_percent_round);
    }

    /** LedSafetyNode::gpioCallback(). */
    void publishEStop(bool pressed)
    {
        blackboard_->set<bool>("e_stop_state", pressed);
        inputs_.e_stop_pressed = pressed;
        updateLedVerdicts();
    }

    /** LedSafetyNode::joyCallback(). */
    void publishDeadMan(bool held)
    {
        blackboard_->set<bool>("drive_state", held);
        inputs_.dead_man_held = held;
        updateLedVerdicts();
    }

    /** One timer period: a tick, then 100 ms. */
    void tick()
    {
        tree_.tickOnce();
        std::this_thread::sleep_for(100ms);
    }

    /** Ticks at 10 Hz for `duration`. */
    void tickFor(std::chrono::milliseconds duration)
    {
        const auto end = std::chrono::steady_clock::now() + duration;
        while (std::chrono::steady_clock::now() < end) {
            tick();
        }
    }

    /** Ticks at 10 Hz until `id` is requested or `timeout` passes. */
    bool tickUntilRequested(unsigned id, std::chrono::milliseconds timeout)
    {
        const auto end = std::chrono::steady_clock::now() + timeout;
        while (std::chrono::steady_clock::now() < end) {
            tick();
            if (std::any_of(calls_.begin(), calls_.end(), [id](const auto & c) { return c.id == id; })) {
                return true;
            }
        }
        return false;
    }

    /** The recorded calls with one of `ids`, in order; clears every recorded call. */
    Calls takeCalls(const std::vector<unsigned> & ids)
    {
        Calls taken;
        std::copy_if(calls_.begin(), calls_.end(), std::back_inserter(taken), [&ids](const auto & c) {
            return std::find(ids.begin(), ids.end(), c.id) != ids.end();
        });
        calls_.clear();
        return taken;
    }

    Calls takeAllCalls()
    {
        Calls taken;
        taken.swap(calls_);
        return taken;
    }

    /** A discharging, idle rover that has shown its first animations. */
    void startIdle(float percentage = 0.9f)
    {
        createTree();
        publishEStop(false);
        publishBattery(
            BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING, BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD,
            percentage);
    }

    Calls calls_;
    BT::BehaviorTreeFactory factory_;
    BT::Blackboard::Ptr blackboard_;
    BT::Tree tree_;

private:
    /** LedSafetyNode::updateLedVerdicts(). */
    domain::LedAnimationDecision updateLedVerdicts()
    {
        const auto decision = domain::evaluateLedAnimation(inputs_, thresholds_);
        blackboard_->set<unsigned>("led_state_verdict", unsigned(decision.state));
        blackboard_->set<unsigned>("led_error_verdict", unsigned(decision.error));
        blackboard_->set<unsigned>("led_battery_verdict", unsigned(decision.battery));
        blackboard_->set<bool>("led_battery_full", decision.battery_full);
        blackboard_->set<bool>("led_battery_low", decision.low_battery);
        blackboard_->set<bool>("led_battery_critical", decision.critical_battery);
        blackboard_->set<bool>("led_battery_nominal", decision.nominal_battery);
        return decision;
    }

    domain::LedSafetyInputs inputs_;
    domain::LedBatteryThresholds thresholds_{kShippedThresholds};
};

}  // namespace

TEST_F(LedSafetyTreeTest, IdleRoverShowsNoErrorNominalAndReadyOnce)
{
    startIdle();

    tick();
    EXPECT_EQ(takeAllCalls(), (Calls{kNoError, kBatteryNominal, kReady}));

    tickFor(1s);
    EXPECT_EQ(takeAllCalls(), Calls{});
}

TEST_F(LedSafetyTreeTest, EStopOverridesDeadMan)
{
    startIdle();
    tickFor(300ms);
    ASSERT_EQ(takeCalls(kStateIds), Calls{kReady});

    publishDeadMan(true);
    tickFor(300ms);
    EXPECT_EQ(takeCalls(kStateIds), Calls{kManualAction});

    publishEStop(true);
    tickFor(300ms);
    EXPECT_EQ(takeCalls(kStateIds), Calls{kEStop});

    publishDeadMan(false);
    tickFor(300ms);
    EXPECT_EQ(takeCalls(kStateIds), Calls{});

    publishEStop(false);
    tickFor(300ms);
    EXPECT_EQ(takeCalls(kStateIds), Calls{kReady});
}

TEST_F(LedSafetyTreeTest, ErrorWhileStatusUnknownOrChargingOverheat)
{
    createTree();
    publishEStop(false);

    publishBattery(
        BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN, BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD, 0.9f);
    tickFor(300ms);
    EXPECT_EQ(takeCalls(kErrorIds), Calls{kError});

    publishBattery(
        BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING, BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD, 0.9f);
    tickFor(300ms);
    EXPECT_EQ(takeCalls(kErrorIds), Calls{kNoError});

    publishBattery(
        BatteryStateMsg::POWER_SUPPLY_STATUS_CHARGING, BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERHEAT,
        0.9f);
    tick();
    EXPECT_EQ(takeCalls(kErrorIds), Calls{kError});

    // FULL counts as charging for the battery animations, not for the overheat error.
    createTree();
    publishEStop(false);
    publishBattery(
        BatteryStateMsg::POWER_SUPPLY_STATUS_FULL, BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERHEAT, 0.9f);
    tickFor(3s);
    EXPECT_EQ(takeCalls(kErrorIds), Calls{kNoError});
}

TEST_F(LedSafetyTreeTest, DischargingSelectsLevelAtThresholds)
{
    startIdle(0.05f);
    tickFor(500ms);
    EXPECT_EQ(takeCalls(kBatteryIds), Calls{call(LedAnimationMsg::CRITICAL_BATTERY, "0.050000", true)});

    // LOW_BATTERY waits one LOW_BATTERY_ANIM_PERIOD after the tree is created.
    startIdle(0.1f);
    EXPECT_TRUE(tickUntilRequested(LedAnimationMsg::LOW_BATTERY, 1s));
    EXPECT_EQ(takeCalls(kBatteryIds), Calls{call(LedAnimationMsg::LOW_BATTERY, "0.100000", false)});

    startIdle(0.4f);
    tickFor(500ms);
    EXPECT_EQ(takeCalls(kBatteryIds), Calls{kBatteryNominal});

    startIdle(std::numeric_limits<float>::quiet_NaN());
    tickFor(500ms);
    EXPECT_EQ(takeCalls(kBatteryIds), Calls{});

    createTree();
    publishEStop(false);
    publishBattery(
        BatteryStateMsg::POWER_SUPPLY_STATUS_NOT_CHARGING, BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD,
        0.05f);
    tickFor(500ms);
    EXPECT_EQ(takeCalls(kBatteryIds), Calls{call(LedAnimationMsg::CRITICAL_BATTERY, "0.050000", true)});
}

TEST_F(LedSafetyTreeTest, LowBatteryRepeatsEveryPeriod)
{
    startIdle(0.2f);
    tickFor(1s);

    const auto calls = takeCalls(kBatteryIds);
    EXPECT_GE(calls.size(), 2u);
    EXPECT_LE(calls.size(), 4u);
    for (const auto & c : calls) {
        EXPECT_EQ(c, call(LedAnimationMsg::LOW_BATTERY, "0.200000", false));
    }
}

TEST_F(LedSafetyTreeTest, ChargingShowsChargerInsertedThenPercentThenCharged)
{
    createTree();
    publishEStop(false);
    publishBattery(
        BatteryStateMsg::POWER_SUPPLY_STATUS_CHARGING, BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD, 0.5f);

    const auto start = std::chrono::steady_clock::now();
    const auto tickUntil = [this, start](std::chrono::milliseconds since_start) {
        while (std::chrono::steady_clock::now() < start + since_start) {
            tick();
        }
    };

    tickUntil(250ms);
    EXPECT_EQ(takeCalls(kBatteryIds), Calls{});
    tickUntil(600ms);
    EXPECT_EQ(takeCalls(kBatteryIds), Calls{kChargerInserted});
    tickUntil(3000ms);
    EXPECT_EQ(takeCalls(kBatteryIds), Calls{call(LedAnimationMsg::CHARGING_BATTERY, "0.500000", true)});
    tickFor(1s);
    EXPECT_EQ(takeCalls(kBatteryIds), Calls{});

    publishBattery(
        BatteryStateMsg::POWER_SUPPLY_STATUS_CHARGING, BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD, 0.56f);
    tickFor(3s);
    EXPECT_EQ(takeCalls(kBatteryIds), Calls{call(LedAnimationMsg::CHARGING_BATTERY, "0.550000", true)});

    publishBattery(
        BatteryStateMsg::POWER_SUPPLY_STATUS_CHARGING, BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD, 1.0f);
    tickFor(3s);
    EXPECT_EQ(takeCalls(kBatteryIds), Calls{call(LedAnimationMsg::BATTERY_CHARGED, "1.000000", true)});
}

// The tree records what it sent within the tick, so with critical > low both levels hold and they
// displace each other every tick.
TEST_F(LedSafetyTreeTest, MisorderedThresholdsResendCriticalAndNominalEveryTick)
{
    createTree({0.5f, 0.3f, 0.05f});
    publishEStop(false);
    publishBattery(
        BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING, BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD, 0.4f);

    for (int i = 0; i < 3; ++i) {
        tick();
        EXPECT_EQ(
            takeCalls(kBatteryIds),
            (Calls{call(LedAnimationMsg::CRITICAL_BATTERY, "0.400000", true), kBatteryNominal}))
            << "tick " << i;
    }
}
