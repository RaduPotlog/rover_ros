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

#include <chrono>

#include "rover_crsf_teleop/domain/safety_io_monitor.hpp"

namespace rover_crsf_teleop
{
namespace
{

using namespace std::chrono_literals;

// The one combination that grants the permit: button pressed, PLC latched, contacts open.
SafetyIoFlags allEngaged()
{
    SafetyIoFlags flags;
    flags.hw_e_stop_user_button = true;
    flags.sw_e_stop_latch_status = true;
    flags.motor_contactor_engaged = false;
    return flags;
}

const SteadyTime kT0{};
constexpr std::chrono::milliseconds kTimeout{1000};

}  // namespace

TEST(SafetyIoMonitorTest, UnknownBeforeAnySample)
{
    const SafetyIoMonitor monitor(kTimeout);

    EXPECT_EQ(monitor.eStopState(kT0), EStopState::kUnknown);
    EXPECT_FALSE(monitor.sampleAge(kT0).has_value());
}

TEST(SafetyIoMonitorTest, AFreshPermittingSampleIsEngaged)
{
    SafetyIoMonitor monitor(kTimeout);
    monitor.onSample(kT0, allEngaged());

    EXPECT_EQ(monitor.eStopState(kT0), EStopState::kEngaged);
    EXPECT_EQ(monitor.eStopState(kT0 + 999ms), EStopState::kEngaged);
}

TEST(SafetyIoMonitorTest, AFreshNonPermittingSampleIsReleased)
{
    SafetyIoMonitor monitor(kTimeout);
    monitor.onSample(kT0, SafetyIoFlags{});
    EXPECT_EQ(monitor.eStopState(kT0), EStopState::kReleased);

    // The software E-Stop alone: latched and open, but the physical button is released.
    SafetyIoFlags software_only;
    software_only.hw_e_stop_user_button = false;
    software_only.sw_e_stop_latch_status = true;
    software_only.motor_contactor_engaged = false;
    monitor.onSample(kT0, software_only);
    EXPECT_EQ(monitor.eStopState(kT0), EStopState::kReleased);
}

// The path that matters: a publisher that has stopped must not keep granting the permit.
TEST(SafetyIoMonitorTest, AnEngagedSampleGoesUnknownOnceStale)
{
    SafetyIoMonitor monitor(kTimeout);
    monitor.onSample(kT0, allEngaged());

    // Exactly the timeout old is still fresh, as in LinkMonitor.
    EXPECT_EQ(monitor.eStopState(kT0 + kTimeout), EStopState::kEngaged);
    EXPECT_EQ(monitor.eStopState(kT0 + kTimeout + SteadyTime::duration{1}), EStopState::kUnknown);
    EXPECT_EQ(monitor.eStopState(kT0 + 10s), EStopState::kUnknown);
}

TEST(SafetyIoMonitorTest, AReleasedSampleAlsoGoesUnknownOnceStale)
{
    SafetyIoMonitor monitor(kTimeout);
    monitor.onSample(kT0, SafetyIoFlags{});

    EXPECT_EQ(monitor.eStopState(kT0 + kTimeout + SteadyTime::duration{1}), EStopState::kUnknown);
}

TEST(SafetyIoMonitorTest, ANewSampleMakesItFreshAgain)
{
    SafetyIoMonitor monitor(kTimeout);
    monitor.onSample(kT0, allEngaged());
    ASSERT_EQ(monitor.eStopState(kT0 + 2s), EStopState::kUnknown);

    monitor.onSample(kT0 + 2s, allEngaged());
    EXPECT_EQ(monitor.eStopState(kT0 + 2s), EStopState::kEngaged);

    // The latest sample wins, whatever the one before it said.
    monitor.onSample(kT0 + 2100ms, SafetyIoFlags{});
    EXPECT_EQ(monitor.eStopState(kT0 + 2100ms), EStopState::kReleased);
}

TEST(SafetyIoMonitorTest, ClearForgetsTheSample)
{
    SafetyIoMonitor monitor(kTimeout);
    monitor.onSample(kT0, allEngaged());

    monitor.clear();

    EXPECT_EQ(monitor.eStopState(kT0), EStopState::kUnknown);
    EXPECT_FALSE(monitor.sampleAge(kT0).has_value());
}

TEST(SafetyIoMonitorTest, SampleAgeIsMillisecondsSinceArrival)
{
    SafetyIoMonitor monitor(kTimeout);
    monitor.onSample(kT0 + 50ms, allEngaged());

    ASSERT_TRUE(monitor.sampleAge(kT0 + 100ms).has_value());
    EXPECT_EQ(*monitor.sampleAge(kT0 + 100ms), 50ms);

    // Still reported once stale: the diagnostic shows how old the evidence is, not just that it
    // is too old.
    ASSERT_TRUE(monitor.sampleAge(kT0 + 5s).has_value());
    EXPECT_EQ(*monitor.sampleAge(kT0 + 5s), 4950ms);
}

TEST(SafetyIoMonitorTest, TheTimeoutComesFromTheConstructor)
{
    SafetyIoMonitor monitor(200ms);
    monitor.onSample(kT0, allEngaged());

    EXPECT_EQ(monitor.eStopState(kT0 + 200ms), EStopState::kEngaged);
    EXPECT_EQ(monitor.eStopState(kT0 + 201ms), EStopState::kUnknown);
}

}  // namespace rover_crsf_teleop
