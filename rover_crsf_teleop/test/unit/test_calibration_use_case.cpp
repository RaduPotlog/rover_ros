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
#include <optional>
#include <string>

#include "rover_crsf_teleop/application/calibration_use_case.hpp"

namespace rover_crsf_teleop
{
namespace
{

constexpr int kLinearChannel = 3;
constexpr int kAngularChannel = 1;
constexpr int kRest = 1004;
constexpr auto kTimeout = std::chrono::seconds(300);
constexpr auto kGrace = std::chrono::milliseconds(1000);

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
        return CalibrationUseCase(
            defaultCalibration(), axisMask(), store, teleop, kTimeout, kGrace);
    }

    // The ordinary case: E-Stop verified engaged and the operator confirming it.
    CalibrationOutcome start(CalibrationUseCase & use_case)
    {
        return use_case.start(true, EStopState::kEngaged, now);
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
        ASSERT_TRUE(use_case.start(true, EStopState::kEngaged, now).ok);
        feedRest(use_case, kCenterSampleTarget);
        ASSERT_TRUE(use_case.beginSweep().ok);
        sweep(use_case);
        ASSERT_TRUE(use_case.finish().ok);
    }
};

// A store that can actually hand something back, for the startup-resolution tests below. The
// FakeStore above deliberately always loads empty, which the session tests depend on.
class LoadableStore : public CalibrationStorePort
{

public:

    std::optional<StoredCalibration> load() override { return stored; }

    bool save(const ChannelCalibration &, std::string &) override { return true; }

    std::string location() const override { return "/config/rc_calibration.yaml"; }

    std::optional<StoredCalibration> stored;
};

TeleopConfig baseConfig()
{
    TeleopConfig config;
    config.linear_x_channel = kLinearChannel;
    config.angular_z_channel = kAngularChannel;
    config.linear_x_mapping.out_max = 1.2;
    config.angular_z_mapping.out_max = 1.0;
    return config;
}

StoredCalibration storedAt(const int linear_mid, const int linear_deadband)
{
    StoredCalibration stored;
    stored.created = "2026-09-20T18:00:00Z";
    stored.calibration = defaultCalibration();
    stored.calibration.in_mid[kLinearChannel - 1] = linear_mid;
    stored.calibration.deadband[kLinearChannel - 1] = linear_deadband;
    return stored;
}

TEST(ResolveStartupCalibrationTest, NoStoreLeavesTheConfiguredParametersInForce)
{
    const TeleopConfig base = baseConfig();

    const StartupCalibration result = resolveStartupCalibration(base, nullptr, axisMask());

    EXPECT_EQ(result.outcome, StartupCalibrationOutcome::kNoStore);
    EXPECT_EQ(result.source, "the configured parameters");
    EXPECT_EQ(result.config.linear_x_mapping.in_mid, base.linear_x_mapping.in_mid);
    EXPECT_TRUE(result.detail.empty());
}

TEST(ResolveStartupCalibrationTest, AnEmptyStoreLeavesTheConfiguredParametersInForce)
{
    LoadableStore store;

    const StartupCalibration result = resolveStartupCalibration(baseConfig(), &store, axisMask());

    EXPECT_EQ(result.outcome, StartupCalibrationOutcome::kNothingStored);
    EXPECT_EQ(result.source, "the configured parameters");
}

TEST(ResolveStartupCalibrationTest, AUsableStoredCalibrationWins)
{
    LoadableStore store;
    store.stored = storedAt(kRest, 14);

    const StartupCalibration result = resolveStartupCalibration(baseConfig(), &store, axisMask());

    EXPECT_EQ(result.outcome, StartupCalibrationOutcome::kStoredApplied);
    EXPECT_EQ(result.config.linear_x_mapping.in_mid, kRest);
    EXPECT_EQ(result.config.linear_x_mapping.deadband_counts, 14);

    // The output limits still come from the parameters.
    EXPECT_DOUBLE_EQ(result.config.linear_x_mapping.out_max, 1.2);

    // Names the file and when it was measured, for the "Calibration in effect" diagnostic.
    EXPECT_NE(result.source.find("/config/rc_calibration.yaml"), std::string::npos);
    EXPECT_NE(result.source.find("2026-09-20T18:00:00Z"), std::string::npos);
}

TEST(ResolveStartupCalibrationTest, AnUnusableStoredCalibrationIsRefusedWhole)
{
    LoadableStore store;

    // A deadband wider than the throw: mapAxis() would return 0.0 for every input, so this axis
    // would be silently dead. Half-applying it is worse than not applying it at all.
    store.stored = storedAt(kRest, 2000);

    const TeleopConfig base = baseConfig();
    const StartupCalibration result = resolveStartupCalibration(base, &store, axisMask());

    EXPECT_EQ(result.outcome, StartupCalibrationOutcome::kStoredRefused);
    EXPECT_FALSE(result.detail.empty()) << "the operator has to be told what was wrong with it";
    EXPECT_EQ(result.source, "the configured parameters");

    // Nothing from the refused calibration reached the config - not even the good channels.
    EXPECT_EQ(result.config.linear_x_mapping.in_mid, base.linear_x_mapping.in_mid);
    EXPECT_EQ(
        result.config.linear_x_mapping.deadband_counts, base.linear_x_mapping.deadband_counts);
    EXPECT_EQ(result.config.calibration.in_mid[kLinearChannel - 1],
              base.calibration.in_mid[kLinearChannel - 1]);
}

TEST(CalibrationUseCaseTest, StartIsRefusedWithoutTheEStopConfirmation)
{
    Fixture fixture;
    auto use_case = fixture.make();

    const CalibrationOutcome outcome = use_case.start(false, EStopState::kEngaged, fixture.now);

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
    EXPECT_FALSE(use_case.start(true, EStopState::kEngaged, fixture.now).ok);
    EXPECT_FALSE(fixture.teleop.inhibited());
}

TEST(CalibrationUseCaseTest, AStartedSessionInhibitsTeleop)
{
    Fixture fixture;
    auto use_case = fixture.make();

    ASSERT_TRUE(use_case.start(true, EStopState::kEngaged, fixture.now).ok);

    EXPECT_TRUE(use_case.sessionInProgress());
    EXPECT_TRUE(fixture.teleop.inhibited());
    EXPECT_FALSE(use_case.start(true, EStopState::kEngaged, fixture.now).ok) << "a second start must be refused";
}

TEST(CalibrationUseCaseTest, CancelReleasesTheInhibit)
{
    Fixture fixture;
    auto use_case = fixture.make();
    ASSERT_TRUE(use_case.start(true, EStopState::kEngaged, fixture.now).ok);

    EXPECT_TRUE(use_case.cancel().ok);
    EXPECT_FALSE(use_case.sessionInProgress());
    EXPECT_FALSE(fixture.teleop.inhibited());
    EXPECT_FALSE(use_case.cancel().ok) << "nothing to cancel twice";
}

TEST(CalibrationUseCaseTest, AnAbandonedSessionTimesOutAndReleasesTheInhibit)
{
    Fixture fixture;
    auto use_case = fixture.make();
    ASSERT_TRUE(use_case.start(true, EStopState::kEngaged, fixture.now).ok);

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
    ASSERT_TRUE(use_case.start(true, EStopState::kEngaged, fixture.now).ok);
    EXPECT_FALSE(use_case.beginSweep().ok) << "not enough frames at rest";

    fixture.feedRest(use_case, kCenterSampleTarget);
    EXPECT_TRUE(use_case.beginSweep().ok);
    EXPECT_FALSE(use_case.finish().ok) << "nothing has moved yet";
}

TEST(CalibrationUseCaseTest, TheSnapshotReportsProblemsOnlyOnceTheMeasurementIsComplete)
{
    Fixture fixture;
    auto use_case = fixture.make();

    ASSERT_TRUE(use_case.start(true, EStopState::kEngaged, fixture.now).ok);
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
    ASSERT_TRUE(use_case.start(true, EStopState::kEngaged, fixture.now).ok);
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

// --- the E-Stop gate ---------------------------------------------------------------------------

TEST(CalibrationUseCaseTest, StartIsRefusedWhileTheEStopIsReleased)
{
    Fixture fixture;
    auto use_case = fixture.make();

    const CalibrationOutcome outcome =
        use_case.start(true, EStopState::kReleased, fixture.now);

    EXPECT_FALSE(outcome.ok);
    EXPECT_NE(outcome.message.find("Press the physical E-Stop"), std::string::npos);
    EXPECT_FALSE(fixture.teleop.inhibited());
}

TEST(CalibrationUseCaseTest, StartIsRefusedWhenTheEStopCannotBeVerified)
{
    Fixture fixture;
    auto use_case = fixture.make();

    // Nothing on gpio_state is not "probably fine". The operator ticking the box does not make
    // the rover safe, which is the whole reason the topic is consulted at all.
    const CalibrationOutcome outcome = use_case.start(true, EStopState::kUnknown, fixture.now);

    EXPECT_FALSE(outcome.ok);
    EXPECT_NE(outcome.message.find("Cannot verify"), std::string::npos);
    EXPECT_FALSE(use_case.sessionInProgress());
}

TEST(CalibrationUseCaseTest, EachRefusalNamesItsOwnCause)
{
    // Whichever gate stops them, the operator has to be told which one - "refused" on its own
    // sends people to the wrong thing.
    Fixture fixture;
    fixture.teleop.could_command = true;
    auto active = fixture.make();
    const std::string active_message =
        active.start(true, EStopState::kEngaged, fixture.now).message;

    Fixture released_fixture;
    auto released = released_fixture.make();
    const std::string released_message =
        released.start(true, EStopState::kReleased, released_fixture.now).message;

    Fixture unconfirmed_fixture;
    auto unconfirmed = unconfirmed_fixture.make();
    const std::string unconfirmed_message =
        unconfirmed.start(false, EStopState::kEngaged, unconfirmed_fixture.now).message;

    EXPECT_NE(active_message, released_message);
    EXPECT_NE(released_message, unconfirmed_message);
    EXPECT_NE(active_message, unconfirmed_message);
}

TEST(CalibrationUseCaseTest, TheSnapshotReportsTheVerifiedEStopNotTheConfirmation)
{
    Fixture fixture;
    auto use_case = fixture.make();

    EXPECT_EQ(use_case.snapshot(fixture.now).e_stop, EStopState::kUnknown);

    use_case.onEStop(EStopState::kEngaged, fixture.now);
    EXPECT_EQ(use_case.snapshot(fixture.now).e_stop, EStopState::kEngaged);

    use_case.onEStop(EStopState::kReleased, fixture.now);
    EXPECT_EQ(use_case.snapshot(fixture.now).e_stop, EStopState::kReleased);
}

TEST(CalibrationUseCaseTest, ReleasingTheEStopCancelsARunningSession)
{
    Fixture fixture;
    auto use_case = fixture.make();
    ASSERT_TRUE(fixture.start(use_case).ok);

    use_case.onEStop(EStopState::kReleased, fixture.now);
    ASSERT_TRUE(use_case.sessionInProgress()) << "not before the grace window has passed";

    use_case.onEStop(EStopState::kReleased, fixture.now + kGrace + std::chrono::milliseconds(1));

    EXPECT_FALSE(use_case.sessionInProgress());
    EXPECT_FALSE(fixture.teleop.inhibited());
    EXPECT_NE(use_case.snapshot(fixture.now).message.find("E-Stop"), std::string::npos);
}

TEST(CalibrationUseCaseTest, ABriefEStopDropoutDoesNotThrowAwayTheMeasurement)
{
    Fixture fixture;
    auto use_case = fixture.make();
    ASSERT_TRUE(fixture.start(use_case).ok);

    // The driver reports a Modbus read error as "clear", and the underlying IO only refreshes at
    // 2 Hz, so a single not-engaged sample is a hiccup - not consent being withdrawn.
    use_case.onEStop(EStopState::kReleased, fixture.now);
    use_case.onEStop(EStopState::kEngaged, fixture.now + std::chrono::milliseconds(200));
    use_case.onEStop(EStopState::kEngaged, fixture.now + kGrace + std::chrono::seconds(5));

    EXPECT_TRUE(use_case.sessionInProgress());
    EXPECT_TRUE(fixture.teleop.inhibited());
}

TEST(CalibrationUseCaseTest, AnUnverifiableEStopAlsoCancelsARunningSession)
{
    Fixture fixture;
    auto use_case = fixture.make();
    ASSERT_TRUE(fixture.start(use_case).ok);

    // The publisher dying mid-sweep is exactly as disqualifying as the button being released:
    // either way nothing can vouch for the rover any more.
    use_case.onEStop(EStopState::kUnknown, fixture.now);
    use_case.onEStop(EStopState::kUnknown, fixture.now + kGrace + std::chrono::milliseconds(1));

    EXPECT_FALSE(use_case.sessionInProgress());
}

TEST(CalibrationUseCaseTest, ApplyIsRefusedOnceTheEStopIsNoLongerEngaged)
{
    Fixture fixture;
    auto use_case = fixture.make();
    fixture.measure(use_case);

    // Inside the grace window, so the session is still open - but applying is a state change and
    // must not happen unsupervised.
    use_case.onEStop(EStopState::kReleased, fixture.now);
    ASSERT_TRUE(use_case.sessionInProgress());

    const CalibrationOutcome outcome = use_case.apply(nullptr, false);

    EXPECT_FALSE(outcome.ok);
    EXPECT_EQ(fixture.teleop.rebuilds, 0);
}

TEST(CalibrationUseCaseTest, TheEStopIsTrackedWhileIdleWithoutStartingAnything)
{
    Fixture fixture;
    auto use_case = fixture.make();

    // The page shows the live state before anyone presses Start, so this has to work at idle -
    // and must not be mistaken for a session.
    use_case.onEStop(EStopState::kReleased, fixture.now);
    use_case.onEStop(EStopState::kReleased, fixture.now + kGrace + std::chrono::seconds(60));

    EXPECT_FALSE(use_case.sessionInProgress());
    EXPECT_FALSE(fixture.teleop.inhibited());
}

}  // namespace
}  // namespace rover_crsf_teleop
