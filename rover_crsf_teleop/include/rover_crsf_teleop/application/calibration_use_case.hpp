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

#include "rover_crsf_teleop/application/teleop_use_case.hpp"
#include "rover_crsf_teleop/domain/link_monitor.hpp"
#include "rover_crsf_teleop/domain/ports.hpp"
#include "rover_crsf_teleop/domain/rc_calibration.hpp"
#include "rover_crsf_teleop/domain/rc_frame.hpp"

namespace rover_crsf_teleop
{

// Which calibration a freshly configured node ended up driving on, and why. The node turns this
// into a log line; nothing here is a ROS type and nothing here logs.
enum class StartupCalibrationOutcome
{
    kNoStore,         // persistence is off - no file was configured
    kNothingStored,   // a store is configured but has nothing in it yet
    kStoredApplied,   // a stored calibration passed its checks and is in force
    kStoredRefused,   // a stored calibration failed its checks and was rejected whole
};

struct StartupCalibration
{
    // The base config, with the winning calibration applied. On kStoredRefused this is the base
    // unchanged - a calibration that is wrong on one channel is refused whole rather than applied
    // in part, because half a calibration is a rover that drives differently on one axis than the
    // operator measured.
    TeleopConfig config;

    // What is in force, phrased for the "Calibration in effect" diagnostic.
    std::string source;

    StartupCalibrationOutcome outcome{StartupCalibrationOutcome::kNoStore};

    // kStoredRefused: the first problem found, ready to drop into a warning. Empty otherwise.
    std::string detail;
};

// Decides which calibration a starting node drives on: a stored one if there is one and it is
// usable, the configured parameters otherwise.
//
// `store` may be null, meaning persistence is off. `axis_channels` flags the channels used as
// proportional axes - only those are checked, because a switch channel legitimately rests at one
// end of its travel (see calibrationProblems).
//
// Does no I/O beyond the port and never logs: the caller owns the wording and the log level.
StartupCalibration resolveStartupCalibration(
    const TeleopConfig & base,
    CalibrationStorePort * store,
    const std::array<bool, RcFrame::kChannelCount> & axis_channels);

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

// What the use case needs the node to do for it. Implemented by the node.
//
// Application rather than domain, and it has to stay there: teleopCouldCommand() asks whether the
// node is lifecycle-ACTIVE, and the lifecycle is a ROS concept the domain is not allowed to know
// about. Moving this next to the ports in domain/ports.hpp for symmetry would drag that concept
// across the boundary - the other three ports describe things the rover does, this one describes
// a state the ROS node is in.
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

    // True while teleop could still command if a frame arrived - the calibration gate. The node
    // answers it from its lifecycle state; that mapping belongs on the implementation, not here.
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
