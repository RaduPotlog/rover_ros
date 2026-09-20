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

#ifndef ROVER_CRSF_TELEOP_APPLICATION_CALIBRATION_USE_CASE_HPP_
#define ROVER_CRSF_TELEOP_APPLICATION_CALIBRATION_USE_CASE_HPP_

#include <array>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rover_crsf_teleop/domain/link_monitor.hpp"
#include "rover_crsf_teleop/domain/ports.hpp"
#include "rover_crsf_teleop/domain/rc_calibration.hpp"
#include "rover_crsf_teleop/domain/rc_frame.hpp"

namespace rover_crsf_teleop
{

// Outcome of one operator request. `message` is what the service response carries back, so it is
// written for a person reading it in a browser, not for a log grep.
struct CalibrationOutcome
{
    bool ok{false};
    std::string message;
};

// Everything a UI or a diagnostic needs about the session, with no ROS types in it.
struct CalibrationSnapshot
{
    CalibrationPhase phase{CalibrationPhase::kIdle};
    unsigned int samples{0};
    double progress{0.0};
    bool teleop_inhibited{false};
    double remaining_s{0.0};

    ChannelCalibration active;
    ChannelCalibration measured;
    std::array<bool, RcFrame::kChannelCount> channel_moved{};
    std::array<int, RcFrame::kChannelCount> latest{};

    std::vector<std::string> problems;
    std::string message;
};

// What the use case needs the node to do for it. Implemented by the node; not a domain port,
// because TeleopConfig is an application type and the collaborator is the same process.
class TeleopControlPort
{

public:

    virtual ~TeleopControlPort() = default;

    // Hold teleop off / let it command again. Releasing MUST also re-arm the switch debouncers:
    // no frames reached them while the inhibit was set, and the sweep moved the switches.
    virtual void setTeleopInhibited(bool inhibited) = 0;

    // Rebuild the teleop rules around `calibration`. Returns false with `reason` filled when it
    // cannot - the node refuses while it is ACTIVE, because rebuilding resets the link monitor.
    virtual bool rebuildTeleop(const ChannelCalibration & calibration, std::string & reason) = 0;

    // True while the node could still command if a frame arrived (lifecycle ACTIVE).
    virtual bool teleopCouldCommand() const = 0;
};

// Drives an RC calibration session on behalf of the operator: safety gate, phases, timeout, and
// what happens to the result.
//
//   start() -> kCenter --beginSweep()--> kSweep --finish()--> kReview --apply()/cancel()--> kIdle
//
// Teleop is inhibited for the whole session and released when it ends, however it ends. The
// switches are re-armed on release: the sweep walks the E-Stop switch through both ends while
// nothing is feeding the debouncers, and without a re-arm the first frame afterwards looks like
// a real edge and fires an E-Stop service call.
class CalibrationUseCase
{

public:

    CalibrationUseCase(
        ChannelCalibration active,
        std::array<bool, RcFrame::kChannelCount> axis_channels,
        std::shared_ptr<CalibrationStorePort> store,
        TeleopControlPort & teleop,
        std::chrono::seconds timeout);

    // Refused unless the operator confirms the E-Stop is engaged AND teleop cannot command. Both
    // are checked here rather than in the UI, so no client can skip either one.
    CalibrationOutcome start(bool e_stop_confirmed, SteadyTime now);

    CalibrationOutcome beginSweep();

    CalibrationOutcome finish();

    CalibrationOutcome cancel();

    // Applies `calibration` (or, when null, the measurement just taken) to the running teleop
    // rules, and with `persist` writes it to the store. A store that refuses the write is a
    // warning in `message`, not a failed apply: the calibration is live either way.
    CalibrationOutcome apply(const ChannelCalibration * calibration, bool persist);

    void onFrame(const RcFrame & frame, SteadyTime now);

    bool sessionInProgress() const { return calibrator_.phase() != CalibrationPhase::kIdle; }

    const ChannelCalibration & active() const { return active_; }

    CalibrationSnapshot snapshot(SteadyTime now) const;

private:

    void endSession(const std::string & message);

    RcCalibrator calibrator_;
    ChannelCalibration active_;
    std::array<bool, RcFrame::kChannelCount> axis_channels_;
    std::shared_ptr<CalibrationStorePort> store_;
    TeleopControlPort & teleop_;
    std::chrono::seconds timeout_;

    std::array<int, RcFrame::kChannelCount> latest_{};
    bool frame_seen_{false};

    // When the session self-cancels. A browser tab closed mid-sweep must not be able to hold
    // teleop inhibited forever.
    SteadyTime deadline_{};

    std::string message_;
};

}  // namespace rover_crsf_teleop

#endif  // ROVER_CRSF_TELEOP_APPLICATION_CALIBRATION_USE_CASE_HPP_
