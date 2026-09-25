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
//
// Unit tests for evaluateSafetyLinkHealth(), the verdict behind the "safety plc link" diagnostic.
// The five summary texts are spelled out here again on purpose, so an accidental edit to one of
// them fails a test instead of silently changing what an operator reads on /diagnostics.

#include <gtest/gtest.h>

#include <cstdint>
#include <string>

#include "rover_hardware_interface/domain/safety_link_diagnosis.hpp"
#include "rover_hardware_interface/domain/safety_link_health.hpp"

namespace rover_hardware_interface
{
namespace test
{

const char * const kHealthyMessage = "Safety PLC link healthy.";
const char * const kFailedOpenMessage =
    "Motor contactor reports open while the E-Stop latch is clear - rover will not drive.";
const char * const kLateHeartbeatMessage =
    "Safety PLC heartbeat is landing late - the link is too slow for the configured margin.";
const char * const kThreadStoppedMessage =
    "A safety controller background thread is not running.";
const char * const kContactorFaultMessage =
    "E-Stop latch asserted but the motor contactor still reports engaged - suspect welded "
    "contacts. Motion inhibited until sw_e_stop_latch_reset and a hardware check.";

// Both background threads up, nothing late - what a started, working link reports.
SafetyLinkHealth healthyLink()
{
    SafetyLinkHealth health;
    health.watchdog_running = true;
    health.poll_running = true;
    health.last_kick_age_ms = 150;
    health.last_poll_age_ms = 40;
    return health;
}

TEST(SafetyLinkDiagnosisTest, DefaultConstructedHealthIsAnError)
{
    // What getHealth() returns before start(): neither thread is running yet.
    const auto d = evaluateSafetyLinkHealth(SafetyLinkHealth{}, false, false);

    EXPECT_EQ(d.severity, SafetyLinkSeverity::kError);
    EXPECT_EQ(d.message, kThreadStoppedMessage);
}

TEST(SafetyLinkDiagnosisTest, HealthyLinkIsOk)
{
    const auto d = evaluateSafetyLinkHealth(healthyLink(), false, false);

    EXPECT_EQ(d.severity, SafetyLinkSeverity::kOk);
    EXPECT_EQ(d.message, kHealthyMessage);
}

TEST(SafetyLinkDiagnosisTest, ContactorFailedOpenIsAWarning)
{
    const auto d = evaluateSafetyLinkHealth(healthyLink(), false, true);

    EXPECT_EQ(d.severity, SafetyLinkSeverity::kWarn);
    EXPECT_EQ(d.message, kFailedOpenMessage);
}

TEST(SafetyLinkDiagnosisTest, OneLateHeartbeatIsAWarning)
{
    auto health = healthyLink();
    health.watchdog_miss_count = 1;

    const auto d = evaluateSafetyLinkHealth(health, false, false);

    EXPECT_EQ(d.severity, SafetyLinkSeverity::kWarn);
    EXPECT_EQ(d.message, kLateHeartbeatMessage);
}

TEST(SafetyLinkDiagnosisTest, LateHeartbeatOutranksContactorFailedOpen)
{
    auto health = healthyLink();
    health.watchdog_miss_count = 3;

    const auto d = evaluateSafetyLinkHealth(health, false, true);

    EXPECT_EQ(d.severity, SafetyLinkSeverity::kWarn);
    EXPECT_EQ(d.message, kLateHeartbeatMessage);
}

TEST(SafetyLinkDiagnosisTest, StoppedWatchdogThreadIsAnError)
{
    auto health = healthyLink();
    health.watchdog_running = false;

    const auto d = evaluateSafetyLinkHealth(health, false, false);

    EXPECT_EQ(d.severity, SafetyLinkSeverity::kError);
    EXPECT_EQ(d.message, kThreadStoppedMessage);
}

TEST(SafetyLinkDiagnosisTest, StoppedPollThreadIsAnError)
{
    auto health = healthyLink();
    health.poll_running = false;

    const auto d = evaluateSafetyLinkHealth(health, false, false);

    EXPECT_EQ(d.severity, SafetyLinkSeverity::kError);
    EXPECT_EQ(d.message, kThreadStoppedMessage);
}

TEST(SafetyLinkDiagnosisTest, StoppedThreadOutranksLateHeartbeatAndFailedOpen)
{
    auto health = healthyLink();
    health.poll_running = false;
    health.watchdog_miss_count = 3;

    const auto d = evaluateSafetyLinkHealth(health, false, true);

    EXPECT_EQ(d.severity, SafetyLinkSeverity::kError);
    EXPECT_EQ(d.message, kThreadStoppedMessage);
}

TEST(SafetyLinkDiagnosisTest, LatchedContactorFaultOnAHealthyLinkIsAnError)
{
    const auto d = evaluateSafetyLinkHealth(healthyLink(), true, false);

    EXPECT_EQ(d.severity, SafetyLinkSeverity::kError);
    EXPECT_EQ(d.message, kContactorFaultMessage);
}

TEST(SafetyLinkDiagnosisTest, LatchedContactorFaultOutranksEverything)
{
    SafetyLinkHealth health;
    health.watchdog_running = false;
    health.poll_running = false;
    health.watchdog_miss_count = 3;

    const auto d = evaluateSafetyLinkHealth(health, true, true);

    EXPECT_EQ(d.severity, SafetyLinkSeverity::kError);
    EXPECT_EQ(d.message, kContactorFaultMessage);
}

// Pins a known gap rather than endorsing it: a link whose every write fails, or that has not
// completed a heartbeat for a minute, still reads OK as long as both threads are alive and no
// tick landed late. Changing that is a behaviour change, not a refactor.
TEST(SafetyLinkDiagnosisTest, ErrorCountersAndAgesNeverRaiseTheLevel)
{
    auto health = healthyLink();
    health.watchdog_error_count = UINT64_MAX;
    health.poll_error_count = UINT64_MAX;

    health.last_kick_age_ms = 60000;
    health.last_poll_age_ms = 60000;
    auto d = evaluateSafetyLinkHealth(health, false, false);
    EXPECT_EQ(d.severity, SafetyLinkSeverity::kOk);
    EXPECT_EQ(d.message, kHealthyMessage);

    health.last_kick_age_ms = SafetyLinkHealth::kUnknownAgeMs;
    health.last_poll_age_ms = SafetyLinkHealth::kUnknownAgeMs;
    d = evaluateSafetyLinkHealth(health, false, false);
    EXPECT_EQ(d.severity, SafetyLinkSeverity::kOk);
    EXPECT_EQ(d.message, kHealthyMessage);
}

TEST(SafetyLinkDiagnosisTest, EveryInputCombinationMatchesThePriorityTable)
{
    for (unsigned bits = 0; bits < 32; ++bits) {
        const bool watchdog_running = (bits & 1u) != 0;
        const bool poll_running = (bits & 2u) != 0;
        const bool late = (bits & 4u) != 0;
        const bool failed_open = (bits & 8u) != 0;
        const bool fault = (bits & 16u) != 0;

        SafetyLinkHealth health;
        health.watchdog_running = watchdog_running;
        health.poll_running = poll_running;
        health.watchdog_miss_count = late ? 3 : 0;

        // Independent oracle: the priority table as a plain if/else chain, highest first.
        SafetyLinkSeverity expected_severity;
        std::string expected_message;
        if (fault) {
            expected_severity = SafetyLinkSeverity::kError;
            expected_message = kContactorFaultMessage;
        } else if (!watchdog_running || !poll_running) {
            expected_severity = SafetyLinkSeverity::kError;
            expected_message = kThreadStoppedMessage;
        } else if (late) {
            expected_severity = SafetyLinkSeverity::kWarn;
            expected_message = kLateHeartbeatMessage;
        } else if (failed_open) {
            expected_severity = SafetyLinkSeverity::kWarn;
            expected_message = kFailedOpenMessage;
        } else {
            expected_severity = SafetyLinkSeverity::kOk;
            expected_message = kHealthyMessage;
        }

        const auto d = evaluateSafetyLinkHealth(health, fault, failed_open);

        EXPECT_EQ(d.severity, expected_severity) << "combo=" << bits;
        EXPECT_EQ(d.message, expected_message) << "combo=" << bits;
    }
}

}  // namespace test
}  // namespace rover_hardware_interface
