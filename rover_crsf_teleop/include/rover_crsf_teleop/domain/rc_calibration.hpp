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

#ifndef ROVER_CRSF_TELEOP_DOMAIN_RC_CALIBRATION_HPP_
#define ROVER_CRSF_TELEOP_DOMAIN_RC_CALIBRATION_HPP_

#include <array>
#include <string>
#include <vector>

#include "rover_crsf_teleop/domain/rc_frame.hpp"
#include "rover_crsf_teleop/domain/stick_mapping.hpp"

namespace rover_crsf_teleop
{

// Raw-count endpoints for every one of the 16 RC channels, indexed 0-based (channel N is index
// N-1), measured from the transmitter that is actually plugged in.
//
// This replaces the single global min/mid/max/deadband the node used to share between both stick
// axes. Sharing them was already wrong in practice: on the A1's transmitter the linear stick
// rests at 1004 and the angular one at 987, 12 and 5 counts either side of the nominal 992, and
// the only way to absorb both with one number was a deadband wide enough for the worse of the
// two. Per channel, each stick gets a deadband sized for its own slop.
struct ChannelCalibration
{
    std::array<int, RcFrame::kChannelCount> in_min{};
    std::array<int, RcFrame::kChannelCount> in_mid{};
    std::array<int, RcFrame::kChannelCount> in_max{};
    std::array<int, RcFrame::kChannelCount> deadband{};
};

// The nominal CRSF endpoints on every channel - what the node uses before anything is measured.
ChannelCalibration defaultCalibration();

// The mapping for one channel (1-16), with the output limits left at their defaults for the
// caller to fill in. Out of range returns the nominal endpoints rather than reading past the
// array: the node validates channel numbers in readConfig() and fails configure on a bad one.
AxisMapping axisMapping(const ChannelCalibration & calibration, int channel_number);

// `mapping` with its endpoints taken from `calibration`, keeping the output limits and inversion
// - those come from the parameters and are not something a calibration measures.
//
// This is the whole of "applying a calibration" to one axis, and it lives next to axisMapping()
// because it is written in terms of it. Everything the calibration owns (in_min/in_mid/in_max and
// the deadband) is replaced; everything the operator configured (out_min/out_max, invert) is not.
AxisMapping mergedMapping(
    const AxisMapping & mapping, const ChannelCalibration & calibration, int channel_number);

// Everything wrong with `calibration` on the channels flagged in `axis_channels`, one
// human-readable sentence each. Empty means usable.
//
// `axis_channels` must name only channels used as *proportional axes* - the sticks. A switch
// channel rests at one end of its travel, not in the middle, so a correctly measured switch has
// in_mid == in_min (or == in_max) and would fail every check here. That is not a fault, so the
// caller passes the mask and switches are simply not checked.
//
// The collapsed-half-throw case is the one that matters: mapAxis() returns 0.0 for a non-positive
// half-span (see stick_mapping.cpp), so a calibration whose deadband swallows one side of the
// throw produces an axis that is silently dead in that direction, with no error anywhere.
std::vector<std::string> calibrationProblems(
    const ChannelCalibration & calibration,
    const std::array<bool, RcFrame::kChannelCount> & axis_channels);

// Frames of rest needed before the centre is trusted, and the target used for progress
// reporting. 50 Hz packet rate, so 50 frames is 1 s and 100 is 2 s.
constexpr unsigned int kMinCenterSamples = 50;
constexpr unsigned int kCenterSampleTarget = 100;

// Added to half the measured rest spread to get the deadband, and the floor it can never go
// under. A stick that reads dead still between frames would otherwise get a deadband of 0, and
// the first count of drift would become a permanent creep command - which in turn makes the
// hardware interface's E-Stop reset invariant ("velocity commands are not zero") unsatisfiable.
constexpr int kDeadbandMargin = 4;
constexpr int kMinDeadband = 8;

// Total travel, in counts, below which a channel counts as "never moved" during the sweep. An
// unassigned channel on the transmitter sits at a constant value; a switch flipped end to end
// covers most of the 1639-count span, so 100 separates them with room to spare.
constexpr int kMinTravel = 100;

enum class CalibrationPhase
{
    kIdle,
    kCenter,
    kSweep,
    kReview,
};

// Measures a transmitter's per-channel endpoints from the frames it sends.
//
// The operator drives the phases; this only accumulates. Centre first (everything released), then
// the sweep (everything moved to both extremes), then the result is available for review:
//
//   start() -> kCenter --beginSweep()--> kSweep --finish()--> kReview --(applied)--> cancel()
//
// cancel() returns to kIdle from anywhere. Pure: no clock, no ROS, no I/O - the use case owns the
// timeout and the node owns the transport.
class RcCalibrator
{

public:

    // Starts a fresh measurement from `previous`, which supplies the values for any channel the
    // sweep never sees move.
    void start(const ChannelCalibration & previous);

    void onFrame(const RcFrame & frame);

    // kCenter -> kSweep. Fails (and stays in kCenter) before kMinCenterSamples frames: a centre
    // averaged over a handful of frames is noise, and everything downstream is defined about it.
    bool beginSweep();

    // kSweep -> kReview, computing the result. Fails if no channel moved at all - that is a
    // transmitter that was never touched, not a calibration.
    bool finish();

    void cancel();

    CalibrationPhase phase() const { return phase_; }

    unsigned int samples() const { return samples_; }

    // Progress through the current phase, 0.0-1.0. The sweep has no defined end - the operator
    // decides when every stick has been moved - so it reports 0.0.
    double progress() const;

    // The measurement as it stands. Meaningful from the first frame of kCenter onwards; complete
    // only in kReview.
    const ChannelCalibration & measured() const { return measured_; }

    const std::array<bool, RcFrame::kChannelCount> & channelsMoved() const { return moved_; }

private:

    void computeCenter();

    CalibrationPhase phase_{CalibrationPhase::kIdle};
    unsigned int samples_{0};

    ChannelCalibration previous_{};
    ChannelCalibration measured_{};
    std::array<bool, RcFrame::kChannelCount> moved_{};

    // Rest-phase accumulators. The sum gives the mean centre, the low/high pair the spread that
    // sizes the deadband: the deadband has to cover where the stick actually rests, not where the
    // protocol says the midpoint is.
    std::array<long, RcFrame::kChannelCount> center_sum_{};
    std::array<int, RcFrame::kChannelCount> center_low_{};
    std::array<int, RcFrame::kChannelCount> center_high_{};

    // Sweep accumulators, seeded from the rest samples so a channel that is never touched still
    // has a self-consistent (if unusable) range.
    std::array<int, RcFrame::kChannelCount> sweep_low_{};
    std::array<int, RcFrame::kChannelCount> sweep_high_{};
};

}  // namespace rover_crsf_teleop

#endif  // ROVER_CRSF_TELEOP_DOMAIN_RC_CALIBRATION_HPP_
