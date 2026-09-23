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
//
// Unit tests for ContactCoilHandler, run against a FakeRoverModbus (no real Modbus TCP
// connection). Assertions are restricted to what's deterministic without synchronizing on the
// background poll thread (see .claude/rules/testing.md's "never sleep(N) to synchronize"):
// the coil-trigger methods are synchronous themselves, and initCoils() runs synchronously inside
// start() before the poll thread is spawned, so both are safe to assert on immediately.

#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <thread>
#include <vector>

#include "rover_hardware_interface/rover_safety_controller/rover_safety_controller.hpp"

#include "fake_rover_modbus.hpp"

namespace rover_hardware_interface
{
namespace test
{


// Waits for `predicate` to hold, polling rather than sleeping a fixed duration, and returns
// whether it became true before `timeout`. Timing tests below still have to let wall-clock time
// pass - they are measuring an interval against a hardware deadline - but no assertion depends
// on a sleep being long enough.
template <typename PredicateT>
bool waitFor(PredicateT predicate, const std::chrono::milliseconds timeout)
{
    const auto deadline = std::chrono::steady_clock::now() + timeout;

    while (std::chrono::steady_clock::now() < deadline) {
        if (predicate()) {
            return true;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    return predicate();
}

// Intervals between consecutive writes to the heartbeat coil (COIL_1), in milliseconds.
std::vector<int64_t> heartbeatIntervalsMs(const std::vector<TimedCoilWrite> & writes)
{
    std::vector<std::chrono::steady_clock::time_point> kicks;

    for (const auto & write : writes) {
        if (write.coil == Coil::COIL_1) {
            kicks.push_back(write.at);
        }
    }

    std::vector<int64_t> intervals;

    for (size_t i = 1; i < kicks.size(); i++) {
        intervals.push_back(
            std::chrono::duration_cast<std::chrono::milliseconds>(kicks[i] - kicks[i - 1]).count());
    }

    return intervals;
}

size_t heartbeatKickCount(const std::vector<TimedCoilWrite> & writes)
{
    return static_cast<size_t>(
        std::count_if(writes.begin(), writes.end(), [](const TimedCoilWrite & write) {
            return write.coil == Coil::COIL_1;
        }));
}

class ContactCoilHandlerTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        modbus = std::make_shared<FakeRoverModbus>();
        handler = std::make_unique<ContactCoilHandler>(modbus);
    }

    std::shared_ptr<FakeRoverModbus> modbus;
    std::unique_ptr<ContactCoilHandler> handler;
};

TEST_F(ContactCoilHandlerTest, StartsDisabled)
{
    EXPECT_FALSE(handler->isContactCoilHandlerEnabled());
}

TEST_F(ContactCoilHandlerTest, EStopUserBtnTriggerWritesCoil2)
{
    handler->eStopUserBtnTrigger(true);

    EXPECT_TRUE(modbus->hasWrite({Coil::COIL_2, true}));
}

TEST_F(ContactCoilHandlerTest, EStopMotorDriverFaultTriggerWritesCoil3)
{
    handler->eStopMotorDriverFaultTrigger(true);

    EXPECT_TRUE(modbus->hasWrite({Coil::COIL_3, true}));
}

TEST_F(ContactCoilHandlerTest, EStopLatchResetPulsesCoil4TrueThenFalse)
{
    handler->eStopLatchReset();

    const auto writes = modbus->writesSnapshot();
    ASSERT_EQ(writes.size(), 2u);
    EXPECT_EQ(writes[0], (CoilWrite{Coil::COIL_4, true}));
    EXPECT_EQ(writes[1], (CoilWrite{Coil::COIL_4, false}));
}

// The reset is a pulse, and the relay can only act on one it is wide enough to see. Before the
// dwell existed the two writes were back-to-back, making the pulse one Modbus round-trip wide -
// a width nobody had checked against the relay's input filter, failing silently if too narrow.
TEST_F(ContactCoilHandlerTest, EStopLatchResetHoldsTheCoilForTheConfiguredDwell)
{
    SafetyControllerSettings settings;
    settings.latch_reset_pulse_ms = 60;

    auto pulse_modbus = std::make_shared<FakeRoverModbus>();
    ContactCoilHandler pulse_handler(pulse_modbus, settings);

    pulse_handler.eStopLatchReset();

    const auto writes = pulse_modbus->timedWritesSnapshot();
    ASSERT_EQ(writes.size(), 2u);

    const auto held_for =
        std::chrono::duration_cast<std::chrono::milliseconds>(writes[1].at - writes[0].at).count();

    EXPECT_GE(held_for, 55) << "latch-reset pulse was only " << held_for << " ms wide";
}

TEST_F(ContactCoilHandlerTest, StartWritesEachWritableCoilsDefaultStateBeforeReturning)
{
    ASSERT_TRUE(handler->start());
    EXPECT_TRUE(handler->isContactCoilHandlerEnabled());

    // Matches coils_config_info_storage_'s default_coil_state for each coil (see
    // rover_safety_controller.cpp) - initCoils() runs synchronously inside start(), before the
    // background threads are spawned, so this is deterministic.
    EXPECT_TRUE(modbus->hasWrite({Coil::COIL_1, true}));
    EXPECT_TRUE(modbus->hasWrite({Coil::COIL_2, true}));
    EXPECT_TRUE(modbus->hasWrite({Coil::COIL_3, true}));
    EXPECT_TRUE(modbus->hasWrite({Coil::COIL_4, false}));
}

// COIL_0 (motor contactor) and COIL_5 (latch status) are relay outputs we only read. This test
// previously asserted the opposite - that initCoils() wrote them - and passed only because the
// fake ignored is_coil_engage_allowed. In production those writes were refused and logged at
// error level on every start.
TEST_F(ContactCoilHandlerTest, StartDoesNotWriteReadOnlyCoils)
{
    ASSERT_TRUE(handler->start());

    EXPECT_FALSE(modbus->hasWrite({Coil::COIL_0, false}));
    EXPECT_FALSE(modbus->hasWrite({Coil::COIL_5, false}));

    // Not merely absent from the accepted writes - never attempted at all, so the real client
    // would have had nothing to reject.
    EXPECT_FALSE(modbus->hasRefusedWrite({Coil::COIL_0, false}));
    EXPECT_FALSE(modbus->hasRefusedWrite({Coil::COIL_5, false}));
}

// The aux outputs come up OFF on every start, so a restart never leaves a load switched on from
// before; the aux inputs are PLC-owned and must never be written, not even refused.
TEST_F(ContactCoilHandlerTest, StartDrivesAuxOutputsOffAndNeverWritesAuxInputs)
{
    ASSERT_TRUE(handler->start());

    for (unsigned i = 0; i < kAuxOutputCount; ++i) {
        const auto coil = static_cast<Coil>(static_cast<unsigned>(Coil::COIL_8) + i);
        EXPECT_TRUE(modbus->hasWrite({coil, false})) << "aux output " << i;
    }

    for (unsigned i = 0; i < kAuxInputCount; ++i) {
        const auto coil = static_cast<Coil>(static_cast<unsigned>(Coil::COIL_14) + i);
        EXPECT_FALSE(modbus->hasWrite({coil, false})) << "aux input " << i;
        EXPECT_FALSE(modbus->hasRefusedWrite({coil, false})) << "aux input " << i;
    }
}

TEST_F(ContactCoilHandlerTest, SetAuxOutputWritesDio00To05)
{
    for (unsigned i = 0; i < kAuxOutputCount; ++i) {
        handler->setAuxOutput(i, true);
    }

    for (unsigned i = 0; i < kAuxOutputCount; ++i) {
        const auto coil = static_cast<Coil>(static_cast<unsigned>(Coil::COIL_8) + i);
        EXPECT_TRUE(modbus->hasWrite({coil, true})) << "aux output " << i;
    }
}

// An index past the last output must not wrap into the aux inputs (COIL_14+) or anything else.
TEST_F(ContactCoilHandlerTest, SetAuxOutputOutOfRangeThrowsAndWritesNothing)
{
    EXPECT_THROW(handler->setAuxOutput(kAuxOutputCount, true), std::out_of_range);
    EXPECT_TRUE(modbus->writesSnapshot().empty());
    EXPECT_TRUE(modbus->refusedWritesSnapshot().empty());
}

TEST_F(ContactCoilHandlerTest, StartIsIdempotent)
{
    ASSERT_TRUE(handler->start());
    EXPECT_TRUE(handler->start());
    EXPECT_TRUE(handler->isContactCoilHandlerEnabled());
}


// --- Heartbeat / IO-poll decoupling -------------------------------------------------------
//
// These are the regression tests for the defect this rework exists to fix. The heartbeat and the
// IO poll used to share one loop: the interval between heartbeat edges was "poll period + every
// Modbus round-trip the poll performed", so a single 500 ms response timeout pushed the toggle
// past the safety relay's ~1 s watchdog window and latched a nuisance E-Stop that then needed a
// manual reset. Written against the old implementation, the first test below fails.

class HeartbeatTimingTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        modbus = std::make_shared<FakeRoverModbus>();
    }

    void startHandler(const unsigned wdg_period_ms, const unsigned poll_period_ms)
    {
        SafetyControllerSettings settings;
        settings.wdg_kick_period_ms = wdg_period_ms;
        settings.io_poll_period_ms = poll_period_ms;

        handler = std::make_unique<ContactCoilHandler>(modbus, settings);
        ASSERT_TRUE(handler->start());
    }

    void TearDown() override
    {
        // Joins both background threads before the fake they reference goes away.
        handler.reset();
    }

    std::shared_ptr<FakeRoverModbus> modbus;
    std::unique_ptr<ContactCoilHandler> handler;
};

TEST_F(HeartbeatTimingTest, HeartbeatHoldsItsPeriodWhileTheIoPollStalls)
{
    // Each read takes 120 ms and the poll period is 10 ms, so the poll thread is effectively
    // holding the link continuously. Under the old single-loop design (7 single-bit reads per
    // sweep) the heartbeat interval would have been ~50 ms + 7 x 120 ms = ~890 ms.
    modbus->setReadDelay(std::chrono::milliseconds(120));

    startHandler(50, 10);

    ASSERT_TRUE(waitFor(
        [this] { return heartbeatKickCount(modbus->timedWritesSnapshot()) >= 6; },
        std::chrono::milliseconds(4000)))
        << "heartbeat did not tick while the IO poll was stalled - it is still serialized behind "
           "the poll's reads";

    const auto intervals = heartbeatIntervalsMs(modbus->timedWritesSnapshot());
    ASSERT_FALSE(intervals.empty());

    const int64_t worst = *std::max_element(intervals.begin(), intervals.end());

    // One in-flight read is the most a tick should ever wait for the link, because the heartbeat
    // acquires the link with priority and the poll therefore cannot start another read while a
    // tick is queued. Measured worst case here is ~120 ms (exactly that one read); 300 ms leaves
    // headroom for a loaded CI box while staying far below both the ~890 ms the old single-loop
    // design produced and the relay's ~1 s window.
    EXPECT_LT(worst, 300)
        << "worst heartbeat interval was " << worst
        << " ms; a stalled IO poll is still delaying the heartbeat";
}

TEST_F(HeartbeatTimingTest, HeartbeatSurvivesIoPollExceptions)
{
    // Every read throws. The poll thread must absorb it rather than let it escape and take the
    // process down with std::terminate(), and the heartbeat must keep running.
    modbus->setFailReadsWithException(true);

    startHandler(30, 10);

    EXPECT_TRUE(waitFor(
        [this] { return heartbeatKickCount(modbus->timedWritesSnapshot()) >= 4; },
        std::chrono::milliseconds(2000)))
        << "heartbeat stopped when the IO poll started throwing";

    EXPECT_GT(handler->getHealth().poll_error_count, 0u);
}

// The sweep cost must stay at three round-trips (the contacts, plus one per PLC coil area)
// however many points are mapped - 19 single-bit reads on a slow link would age the poll past
// the staleness bound safety_status is judged by.
TEST_F(HeartbeatTimingTest, OneIoSweepCostsThreeReadTransactions)
{
    // Long enough that a second sweep cannot start before the count is taken; short enough that
    // TearDown's join does not wait out a long poll sleep.
    startHandler(1000, 2000);

    ASSERT_TRUE(waitFor(
        [this] { return handler->getHealth().last_poll_age_ms != SafetyLinkHealth::kUnknownAgeMs; },
        std::chrono::milliseconds(2000)));

    EXPECT_EQ(modbus->readTransactionCount(), 3u);
}

// Aux writes share the one link with the heartbeat but take it without priority, so hammering
// them - on top of a stalled poll - must not stretch the heartbeat interval.
TEST_F(HeartbeatTimingTest, HeartbeatHoldsItsPeriodWhileAuxWritesContend)
{
    modbus->setReadDelay(std::chrono::milliseconds(120));

    startHandler(50, 10);

    std::atomic_bool stop {false};
    std::thread hammer([this, &stop] {
        bool level = false;
        while (!stop) {
            handler->setAuxOutput(0, level);
            level = !level;
        }
    });

    const bool ticked = waitFor(
        [this] { return heartbeatKickCount(modbus->timedWritesSnapshot()) >= 6; },
        std::chrono::milliseconds(4000));

    stop = true;
    hammer.join();

    ASSERT_TRUE(ticked) << "heartbeat starved by aux output writes";

    const auto intervals = heartbeatIntervalsMs(modbus->timedWritesSnapshot());
    ASSERT_FALSE(intervals.empty());

    const int64_t worst = *std::max_element(intervals.begin(), intervals.end());

    EXPECT_LT(worst, 300) << "worst heartbeat interval was " << worst << " ms";
}

TEST_F(HeartbeatTimingTest, HeartbeatTogglesRatherThanRepeatingALevel)
{
    startHandler(20, 1000);

    ASSERT_TRUE(waitFor(
        [this] { return heartbeatKickCount(modbus->timedWritesSnapshot()) >= 4; },
        std::chrono::milliseconds(2000)));

    // The relay watches for a *changing* level, so consecutive kicks must alternate. A heartbeat
    // that rewrote the same level would satisfy a naive "is it still writing?" check and still
    // let the relay time out.
    std::vector<bool> levels;

    for (const auto & write : modbus->timedWritesSnapshot()) {
        if (write.coil == Coil::COIL_1) {
            levels.push_back(write.state);
        }
    }

    ASSERT_GE(levels.size(), 4u);

    for (size_t i = 1; i < levels.size(); i++) {
        EXPECT_NE(levels[i], levels[i - 1]) << "heartbeat repeated a level at index " << i;
    }
}

TEST_F(HeartbeatTimingTest, ReportsHealthOnceRunning)
{
    startHandler(20, 20);

    ASSERT_TRUE(waitFor(
        [this] {
            const auto health = handler->getHealth();
            return health.last_kick_age_ms != SafetyLinkHealth::kUnknownAgeMs &&
                   health.last_poll_age_ms != SafetyLinkHealth::kUnknownAgeMs;
        },
        std::chrono::milliseconds(2000)));

    const auto health = handler->getHealth();

    EXPECT_TRUE(health.watchdog_running);
    EXPECT_TRUE(health.poll_running);
    EXPECT_EQ(health.watchdog_error_count, 0u);
    EXPECT_EQ(health.poll_error_count, 0u);
}

}  // namespace test
}  // namespace rover_hardware_interface
