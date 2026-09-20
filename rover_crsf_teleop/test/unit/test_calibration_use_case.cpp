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

#include <memory>

#include "rover_crsf_teleop/application/calibration_use_case.hpp"

namespace rover_crsf_teleop
{
namespace
{

constexpr int kLinearChannel = 3;
constexpr int kAngularChannel = 1;
constexpr int kRest = 1004;
constexpr auto kTimeout = std::chrono::seconds(300);

class FakeTeleop : public TeleopControlPort
{

public:

    void setTeleopInhibited(const bool inhibited) override
    {
        inhibited_ = inhibited;
        ++inhibit_changes;
    }

    bool rebuildTeleop(const ChannelCalibration & calibration, std::string & reason) override
    {
        if (!can_rebuild) {
            reason = "the node is active";
            return false;
        }

        rebuilt = calibration;
        ++rebuilds;
        return true;
    }

    bool teleopCouldCommand() const override { return could_command; }

    bool inhibited() const { return inhibited_; }

    bool could_command{false};
    bool can_rebuild{true};
    int rebuilds{0};
    int inhibit_changes{0};
    ChannelCalibration rebuilt{};

private:

    bool inhibited_{false};
};

class FakeStore : public CalibrationStorePort
{

public:

    std::optional<StoredCalibration> load() override { return std::nullopt; }

    bool save(const ChannelCalibration & calibration, std::string & error) override
    {
        ++save_attempts;

        if (!writable) {
            error = "read-only filesystem";
            return false;
        }

        saved = calibration;
        return true;
    }

    std::string location() const override { return "/config/rc_calibration.yaml"; }

    bool writable{true};
    int save_attempts{0};
    ChannelCalibration saved{};
};

std::array<bool, RcFrame::kChannelCount> axisMask()
{
    std::array<bool, RcFrame::kChannelCount> mask{};
    mask[kLinearChannel - 1] = true;
    mask[kAngularChannel - 1] = true;
    return mask;
}

RcFrame frameAt(const int value)
{
    RcFrame frame;
    frame.channels.fill(value);
    return frame;
}

struct Fixture
{
    FakeTeleop teleop;
    std::shared_ptr<FakeStore> store{std::make_shared<FakeStore>()};
    SteadyTime now{SteadyTime::clock::now()};

    CalibrationUseCase make()
    {
        return CalibrationUseCase(defaultCalibration(), axisMask(), store, teleop, kTimeout);
    }

    void feedRest(CalibrationUseCase & use_case, const unsigned int count)
    {
        for (unsigned int i = 0; i < count; ++i) {
            RcFrame frame = frameAt(kRest + ((i % 2 == 0) ? 4 : -4));
            use_case.onFrame(frame, now);
        }
    }

    void sweep(CalibrationUseCase & use_case)
    {
        for (const int value : {172, 1811, kRest}) {
            use_case.onFrame(frameAt(value), now);
        }
    }

    // Carries a session all the way to kReview.
    void measure(CalibrationUseCase & use_case)
    {
        ASSERT_TRUE(use_case.start(true, now).ok);
        feedRest(use_case, kCenterSampleTarget);
        ASSERT_TRUE(use_case.beginSweep().ok);
        sweep(use_case);
        ASSERT_TRUE(use_case.finish().ok);
    }
};

TEST(CalibrationUseCaseTest, StartIsRefusedWithoutTheEStopConfirmation)
{
    Fixture fixture;
    auto use_case = fixture.make();

    const CalibrationOutcome outcome = use_case.start(false, fixture.now);

    EXPECT_FALSE(outcome.ok);
    EXPECT_FALSE(outcome.message.empty());
    EXPECT_FALSE(use_case.sessionInProgress());
    EXPECT_FALSE(fixture.teleop.inhibited());
}

TEST(CalibrationUseCaseTest, StartIsRefusedWhileTeleopCouldStillCommand)
{
    Fixture fixture;
    fixture.teleop.could_command = true;
    auto use_case = fixture.make();

    // The sweep goes to full throw; on an active node that is a full-speed command. The gate is
    // here rather than in the UI so no client can skip it.
    EXPECT_FALSE(use_case.start(true, fixture.now).ok);
    EXPECT_FALSE(fixture.teleop.inhibited());
}

TEST(CalibrationUseCaseTest, AStartedSessionInhibitsTeleop)
{
    Fixture fixture;
    auto use_case = fixture.make();

    ASSERT_TRUE(use_case.start(true, fixture.now).ok);

    EXPECT_TRUE(use_case.sessionInProgress());
    EXPECT_TRUE(fixture.teleop.inhibited());
    EXPECT_FALSE(use_case.start(true, fixture.now).ok) << "a second start must be refused";
}

TEST(CalibrationUseCaseTest, CancelReleasesTheInhibit)
{
    Fixture fixture;
    auto use_case = fixture.make();
    ASSERT_TRUE(use_case.start(true, fixture.now).ok);

    EXPECT_TRUE(use_case.cancel().ok);
    EXPECT_FALSE(use_case.sessionInProgress());
    EXPECT_FALSE(fixture.teleop.inhibited());
    EXPECT_FALSE(use_case.cancel().ok) << "nothing to cancel twice";
}

TEST(CalibrationUseCaseTest, AnAbandonedSessionTimesOutAndReleasesTheInhibit)
{
    Fixture fixture;
    auto use_case = fixture.make();
    ASSERT_TRUE(use_case.start(true, fixture.now).ok);

    // A browser tab closed mid-sweep must not be able to hold teleop off forever.
    use_case.onFrame(frameAt(kRest), fixture.now + kTimeout + std::chrono::seconds(1));

    EXPECT_FALSE(use_case.sessionInProgress());
    EXPECT_FALSE(fixture.teleop.inhibited());
}

TEST(CalibrationUseCaseTest, ApplyRebuildsTeleopAndReleasesTheInhibit)
{
    Fixture fixture;
    auto use_case = fixture.make();
    fixture.measure(use_case);

    const CalibrationOutcome outcome = use_case.apply(nullptr, false);

    ASSERT_TRUE(outcome.ok) << outcome.message;
    EXPECT_EQ(fixture.teleop.rebuilds, 1);
    EXPECT_EQ(fixture.teleop.rebuilt.in_mid[kLinearChannel - 1], kRest);
    EXPECT_EQ(use_case.active().in_mid[kLinearChannel - 1], kRest);
    EXPECT_FALSE(fixture.teleop.inhibited());
    EXPECT_EQ(fixture.store->save_attempts, 0) << "persist was not asked for";
}

TEST(CalibrationUseCaseTest, ApplyWithPersistWritesToTheStore)
{
    Fixture fixture;
    auto use_case = fixture.make();
    fixture.measure(use_case);

    ASSERT_TRUE(use_case.apply(nullptr, true).ok);

    EXPECT_EQ(fixture.store->save_attempts, 1);
    EXPECT_EQ(fixture.store->saved.in_mid[kLinearChannel - 1], kRest);
}

TEST(CalibrationUseCaseTest, AFailedSaveStillAppliesTheCalibrationLive)
{
    Fixture fixture;
    fixture.store->writable = false;
    auto use_case = fixture.make();
    fixture.measure(use_case);

    const CalibrationOutcome outcome = use_case.apply(nullptr, true);

    // A read-only volume must not throw away a measurement that took the operator minutes.
    EXPECT_TRUE(outcome.ok);
    EXPECT_EQ(fixture.teleop.rebuilds, 1);
    EXPECT_NE(outcome.message.find("read-only filesystem"), std::string::npos);
}

TEST(CalibrationUseCaseTest, ApplyIsRefusedWhenTeleopCannotBeRebuilt)
{
    Fixture fixture;
    fixture.teleop.can_rebuild = false;
    auto use_case = fixture.make();
    fixture.measure(use_case);

    const CalibrationOutcome outcome = use_case.apply(nullptr, false);

    EXPECT_FALSE(outcome.ok);
    EXPECT_TRUE(use_case.sessionInProgress()) << "the measurement is kept so it can be retried";
    EXPECT_TRUE(fixture.teleop.inhibited());
}

TEST(CalibrationUseCaseTest, ApplyIsRefusedForADegenerateCalibration)
{
    Fixture fixture;
    auto use_case = fixture.make();
    fixture.measure(use_case);

    ChannelCalibration broken = defaultCalibration();
    broken.deadband[kLinearChannel - 1] = 5000;

    const CalibrationOutcome outcome = use_case.apply(&broken, false);

    EXPECT_FALSE(outcome.ok);
    EXPECT_EQ(fixture.teleop.rebuilds, 0);
}

TEST(CalibrationUseCaseTest, ApplyBeforeAnythingIsMeasuredIsRefused)
{
    Fixture fixture;
    auto use_case = fixture.make();

    EXPECT_FALSE(use_case.apply(nullptr, false).ok);
    EXPECT_EQ(fixture.teleop.rebuilds, 0);
}

TEST(CalibrationUseCaseTest, TheSweepMustFollowTheCentre)
{
    Fixture fixture;
    auto use_case = fixture.make();

    EXPECT_FALSE(use_case.beginSweep().ok) << "no session yet";
    ASSERT_TRUE(use_case.start(true, fixture.now).ok);
    EXPECT_FALSE(use_case.beginSweep().ok) << "not enough frames at rest";

    fixture.feedRest(use_case, kCenterSampleTarget);
    EXPECT_TRUE(use_case.beginSweep().ok);
    EXPECT_FALSE(use_case.finish().ok) << "nothing has moved yet";
}

TEST(CalibrationUseCaseTest, TheSnapshotReportsProblemsOnlyOnceTheMeasurementIsComplete)
{
    Fixture fixture;
    auto use_case = fixture.make();

    ASSERT_TRUE(use_case.start(true, fixture.now).ok);
    fixture.feedRest(use_case, kCenterSampleTarget);
    ASSERT_TRUE(use_case.beginSweep().ok);

    // Mid-sweep the range is still filling in, so every half-throw looks collapsed. Warning the
    // operator about problems that fix themselves a second later is worse than saying nothing.
    use_case.onFrame(frameAt(kRest), fixture.now);
    EXPECT_TRUE(use_case.snapshot(fixture.now).problems.empty());
    EXPECT_TRUE(use_case.snapshot(fixture.now).teleop_inhibited);

    fixture.sweep(use_case);
    ASSERT_TRUE(use_case.finish().ok);
    EXPECT_TRUE(use_case.snapshot(fixture.now).problems.empty());
}

TEST(CalibrationUseCaseTest, AnAxisThatWasNeverSweptIsReported)
{
    Fixture fixture;
    auto use_case = fixture.make();
    ASSERT_TRUE(use_case.start(true, fixture.now).ok);
    fixture.feedRest(use_case, kCenterSampleTarget);
    ASSERT_TRUE(use_case.beginSweep().ok);

    // Only the linear stick moves; the angular one is forgotten.
    for (const int value : {172, 1811, kRest}) {
        RcFrame frame = frameAt(kRest);
        frame.channels[kLinearChannel - 1] = value;
        use_case.onFrame(frame, fixture.now);
    }

    ASSERT_TRUE(use_case.finish().ok);

    const auto problems = use_case.snapshot(fixture.now).problems;
    ASSERT_EQ(problems.size(), 1u);
    EXPECT_NE(problems.front().find("Channel 1"), std::string::npos);
}

TEST(CalibrationUseCaseTest, TheLatestFrameIsReportedEvenWhenNoSessionIsRunning)
{
    Fixture fixture;
    auto use_case = fixture.make();

    // The UI's channel bars come from this, so they work before anyone calibrates anything.
    use_case.onFrame(frameAt(1234), fixture.now);

    const CalibrationSnapshot snapshot = use_case.snapshot(fixture.now);
    EXPECT_EQ(snapshot.latest[0], 1234);
    EXPECT_EQ(snapshot.phase, CalibrationPhase::kIdle);
    EXPECT_FALSE(snapshot.teleop_inhibited);
}

}  // namespace
}  // namespace rover_crsf_teleop
