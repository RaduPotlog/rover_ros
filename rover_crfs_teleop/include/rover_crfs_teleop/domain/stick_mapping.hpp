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

#ifndef ROVER_CRFS_TELEOP_DOMAIN_STICK_MAPPING_HPP_
#define ROVER_CRFS_TELEOP_DOMAIN_STICK_MAPPING_HPP_

namespace rover_crfs_telop
{

// Raw CRSF channel range, as decoded by crsf_receiver and republished unscaled on rc/channels
// (see crsf_receiver/include/crsf_protocol.h). These are the defaults for the corresponding ROS
// parameters, not hard limits - a transmitter with different endpoints can override them.
constexpr int kDefaultCrsfChannelMin = 172;
constexpr int kDefaultCrsfChannelMid = 992;
constexpr int kDefaultCrsfChannelMax = 1811;

// ~3.7 % of the 819-count half-throw. Comfortably covers the measured 12-count resting offset of
// the A1's linear stick with margin for drift, while costing only ~0.07 m/s of resolution around
// centre. Standard RC practice is a 2-5 % centre deadband.
constexpr int kDefaultChannelDeadband = 30;

// How one RC stick axis maps onto one physical command axis.
//
// The mapping is defined about `in_mid`, not across the full span, because the property that
// matters is that a centred stick produces *exactly* 0.0. The previous implementation mapped
// [0, 2000] -> [out_min, out_max] linearly, which put the real centre (992) at -0.016 m/s: the
// rover was permanently commanded to creep, and because a non-zero velocity command blocks the
// hardware interface's E-Stop reset, the E-Stop could never be cleared.
struct AxisMapping
{
    int in_min{kDefaultCrsfChannelMin};
    int in_mid{kDefaultCrsfChannelMid};
    int in_max{kDefaultCrsfChannelMax};

    double out_min{-1.0};
    double out_max{1.0};

    // Half-width, in raw channel counts, of the band around `in_mid` that maps to exactly 0.0.
    // Absorbs stick slop, transmitter trim and per-frame jitter.
    //
    // This has to be sized against where the sticks actually rest, not against frame noise. On
    // the A1's transmitter the linear stick rests at 1004 and the angular one at 987 - 12 and 5
    // counts off the nominal 992 midpoint - which is ordinary centring tolerance, not a fault.
    // A deadband narrower than that leaves a permanent creep command, which in turn makes the
    // hardware interface's E-Stop reset invariant unsatisfiable. See kDefaultChannelDeadband.
    int deadband_counts{0};

    // Negates the normalized stick deflection before the output limits are applied, for an axis
    // whose stick direction is opposite to the robot's sign convention.
    bool invert{false};
};

// Maps a raw channel count to a physical command.
//
// Guarantees:
//   - `in_mid` (and anything within `deadband_counts` of it) returns exactly 0.0;
//   - `in_min`/`in_max` return `out_min`/`out_max` (swapped when `invert`);
//   - values outside [in_min, in_max] are clamped, so a glitched frame can't exceed the limits;
//   - a degenerate mapping (zero or negative half-span, e.g. a deadband wider than the throw)
//     returns 0.0 rather than dividing by zero.
double mapAxis(const int raw_value, const AxisMapping & mapping);

}  // namespace rover_crfs_telop

#endif  // ROVER_CRFS_TELEOP_DOMAIN_STICK_MAPPING_HPP_
