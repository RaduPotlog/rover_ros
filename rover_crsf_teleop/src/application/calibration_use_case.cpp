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

#include "rover_crsf_teleop/application/calibration_use_case.hpp"

#include <utility>

namespace rover_crsf_teleop
{

StartupCalibration resolveStartupCalibration(
    const TeleopConfig & base,
    CalibrationStorePort * const store,
    const std::array<bool, RcFrame::kChannelCount> & axis_channels)
{
    StartupCalibration result;
    result.config = base;
    result.source = "the configured parameters";

    if (store == nullptr) {
        result.outcome = StartupCalibrationOutcome::kNoStore;
        return result;
    }

    const std::optional<StoredCalibration> stored = store->load();

    if (!stored.has_value()) {
        result.outcome = StartupCalibrationOutcome::kNothingStored;
        return result;
    }

    const std::vector<std::string> problems =
        calibrationProblems(stored->calibration, axis_channels);

    if (!problems.empty()) {
        // Refused whole, not in part: the rover keeps driving on the shipped values and the
        // operator is told to re-measure. `config` is deliberately left as `base`.
        result.outcome = StartupCalibrationOutcome::kStoredRefused;
        result.detail = problems.front();
        return result;
    }

    // A calibration measured on this rover describes the transmitter that is actually plugged in,
    // so it wins over the shipped defaults.
    result.config = applyCalibration(base, stored->calibration);
    result.outcome = StartupCalibrationOutcome::kStoredApplied;
    result.detail = stored->created;
    result.source =
        "'" + store->location() + "'" + (stored->created.empty() ? "" : " of " + stored->created);
    return result;
}

CalibrationUseCase::CalibrationUseCase(
    ChannelCalibration active,
    std::array<bool, RcFrame::kChannelCount> axis_channels,
    std::shared_ptr<CalibrationStorePort> store,
    TeleopControlPort & teleop,
    const std::chrono::seconds timeout,
    const std::chrono::milliseconds e_stop_grace)
: active_(std::move(active)),
  axis_channels_(axis_channels),
  store_(std::move(store)),
  teleop_(teleop),
  timeout_(timeout),
  e_stop_grace_(e_stop_grace)
{
}

namespace
{

// One sentence saying why an unverified or released E-Stop blocks a calibration, phrased for
// whoever is standing at the rover rather than for a log grep.
std::string eStopRefusal(const EStopState e_stop)
{
    if (e_stop == EStopState::kReleased) {
        return "Engage the E-Stop before calibrating: the sweep drives the sticks to full throw, "
               "and RC teleop is not the only thing that can command this rover.";
    }

    return "Cannot verify the E-Stop: nothing recent on hardware_interface/safety_status. Is "
           "rover_hardware_interface running? Calibration is refused rather than assumed safe.";
}

}  // namespace

CalibrationOutcome CalibrationUseCase::start(
    const bool e_stop_confirmed, const EStopState e_stop, const SteadyTime now)
{
    if (sessionInProgress()) {
        return {false, "A calibration is already in progress. Cancel it first."};
    }

    // All three gates are checked here, not in the caller: the sweep drives the sticks to full
    // throw, which on an active node is a full-speed command. Ordered so the operator is told
    // about the condition they are most likely to be able to fix first.
    if (teleop_.teleopCouldCommand()) {
        return {false, "Deactivate rover_crsf_teleop_node before starting a calibration: while "
                       "it is active the sweep would command full speed."};
    }

    if (e_stop != EStopState::kEngaged) {
        return {false, eStopRefusal(e_stop)};
    }

    // Evidence and assertion are independent: the rover saying the E-Stop is engaged does not
    // mean anyone is standing next to it, which is what the confirmation is for.
    if (!e_stop_confirmed) {
        return {false, "Confirm the E-Stop is engaged before starting a calibration."};
    }

    calibrator_.start(active_);
    teleop_.setTeleopInhibited(true);
    deadline_ = now + timeout_;
    e_stop_ = e_stop;
    e_stop_lost_at_.reset();
    message_ = "Release every stick and leave the transmitter untouched.";

    return {true, message_};
}

void CalibrationUseCase::onEStop(const EStopState e_stop, const SteadyTime now)
{
    e_stop_ = e_stop;

    if (!sessionInProgress()) {
        e_stop_lost_at_.reset();
        return;
    }

    if (e_stop == EStopState::kEngaged) {
        e_stop_lost_at_.reset();
        return;
    }

    if (!e_stop_lost_at_.has_value()) {
        e_stop_lost_at_ = now;
        return;
    }

    // Only after the grace window. The driver reports a Modbus read error as "clear" and the
    // underlying IO refreshes at 2 Hz, so one not-engaged sample is a hiccup, not consent being
    // withdrawn - and cancelling on it would throw away a measurement that took minutes.
    if ((now - *e_stop_lost_at_) < e_stop_grace_) {
        return;
    }

    endSession(
        "Calibration cancelled: the E-Stop is no longer engaged. The previous values are still "
        "in effect.");
}

CalibrationOutcome CalibrationUseCase::beginSweep()
{
    if (calibrator_.phase() != CalibrationPhase::kCenter) {
        return {false, "Not sampling the centre - start a calibration first."};
    }

    if (!calibrator_.beginSweep()) {
        return {false, "Not enough frames at rest yet. Keep the sticks released for a moment "
                       "longer."};
    }

    message_ = "Move every stick and switch to both extremes, then finish.";
    return {true, message_};
}

CalibrationOutcome CalibrationUseCase::finish()
{
    if (calibrator_.phase() != CalibrationPhase::kSweep) {
        return {false, "Not sweeping - begin the sweep first."};
    }

    if (!calibrator_.finish()) {
        return {false, "No channel moved during the sweep. Is the transmitter on and bound?"};
    }

    // An axis that was never swept keeps its old range, which is usually not what the operator
    // meant to do - say so rather than quietly applying the previous calibration to it.
    std::vector<std::string> problems =
        calibrationProblems(calibrator_.measured(), axis_channels_);
    for (std::size_t i = 0; i < RcFrame::kChannelCount; ++i) {
        if (axis_channels_[i] && !calibrator_.channelsMoved()[i]) {
            problems.push_back(
                "Channel " + std::to_string(i + 1) +
                " drives an axis but never moved during the sweep, so it keeps its previous "
                "range.");
        }
    }

    message_ = problems.empty() ? "Measurement complete." : "Measurement complete, with warnings.";
    return {true, message_};
}

CalibrationOutcome CalibrationUseCase::cancel()
{
    if (!sessionInProgress()) {
        return {false, "No calibration is in progress."};
    }

    endSession("Calibration cancelled; the previous values are still in effect.");
    return {true, message_};
}

CalibrationOutcome CalibrationUseCase::apply(
    const ChannelCalibration * calibration, const bool persist)
{
    if (calibrator_.phase() != CalibrationPhase::kReview && calibration == nullptr) {
        return {false, "Nothing measured to apply - finish a sweep first."};
    }

    const ChannelCalibration requested =
        (calibration != nullptr) ? *calibration : calibrator_.measured();

    const std::vector<std::string> problems = calibrationProblems(requested, axis_channels_);
    if (!problems.empty()) {
        return {false, "Refusing to apply: " + problems.front()};
    }

    // Belt and braces: a released E-Stop should already have cancelled the session, so reaching
    // here means either the grace window has not expired yet or something bypassed onEStop.
    if (e_stop_ != EStopState::kEngaged) {
        return {false, eStopRefusal(e_stop_)};
    }

    std::string reason;
    if (!teleop_.rebuildTeleop(requested, reason)) {
        return {false, "Could not apply the calibration: " + reason};
    }

    active_ = requested;

    std::string message = "Calibration applied.";

    if (persist) {
        if (!store_) {
            message += " It was NOT saved: persistence is off (calibration_file is empty), so it "
                       "is lost on restart.";
        } else {
            std::string error;
            if (store_->save(requested, error)) {
                message += " Saved to " + store_->location() + ".";
            } else {
                // Live is what matters; a failed write must not throw away a measurement the
                // operator just spent minutes taking.
                message += " It is live, but could NOT be saved to " + store_->location() + ": " +
                           error + ". It will be lost on restart.";
            }
        }
    }

    endSession(message);
    return {true, message};
}

void CalibrationUseCase::onFrame(const RcFrame & frame, const SteadyTime now)
{
    latest_ = frame.channels;
    frame_seen_ = true;

    if (!sessionInProgress()) {
        return;
    }

    if (now >= deadline_) {
        endSession("Calibration timed out and was cancelled; the previous values are still in "
                   "effect.");
        return;
    }

    calibrator_.onFrame(frame);

    // kCenter has a target rather than an end: let the operator see it is ready before they act.
    if (calibrator_.phase() == CalibrationPhase::kCenter &&
        calibrator_.samples() == kCenterSampleTarget)
    {
        message_ = "Centre captured. Now sweep every stick and switch to both extremes.";
    }
}

void CalibrationUseCase::endSession(const std::string & message)
{
    calibrator_.cancel();
    teleop_.setTeleopInhibited(false);
    e_stop_lost_at_.reset();
    message_ = message;
}

CalibrationSnapshot CalibrationUseCase::snapshot(const SteadyTime now) const
{
    CalibrationSnapshot snapshot;
    snapshot.phase = calibrator_.phase();
    snapshot.samples = calibrator_.samples();
    snapshot.progress = calibrator_.progress();
    snapshot.teleop_inhibited = sessionInProgress();
    snapshot.e_stop = e_stop_;
    snapshot.active = active_;
    snapshot.measured = calibrator_.measured();
    snapshot.channel_moved = calibrator_.channelsMoved();
    snapshot.latest = latest_;
    snapshot.message = message_;

    if (sessionInProgress()) {
        const auto remaining = std::chrono::duration<double>(deadline_ - now).count();
        snapshot.remaining_s = (remaining > 0.0) ? remaining : 0.0;
    }

    // Only once the measurement is complete. Mid-sweep the range is still filling in, so every
    // half-throw looks collapsed and the operator would be shown warnings that fix themselves.
    if (calibrator_.phase() == CalibrationPhase::kReview) {
        snapshot.problems = calibrationProblems(calibrator_.measured(), axis_channels_);
        for (std::size_t i = 0; i < RcFrame::kChannelCount; ++i) {
            if (axis_channels_[i] && !calibrator_.channelsMoved()[i]) {
                snapshot.problems.push_back(
                    "Channel " + std::to_string(i + 1) +
                    " drives an axis but never moved during the sweep, so it keeps its previous "
                    "range.");
            }
        }
    }

    return snapshot;
}

}  // namespace rover_crsf_teleop
