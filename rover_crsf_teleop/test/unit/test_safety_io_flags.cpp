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

#include "rover_crsf_teleop/domain/safety_io_flags.hpp"

namespace rover_crsf_teleop
{
namespace
{

TEST(SafetyIoFlagsTest, NothingActiveMeansMotionIsNotInhibited)
{
    EXPECT_FALSE(motionIsInhibited(SafetyIoFlags{}));
}

TEST(SafetyIoFlagsTest, AnyOneStopInhibitsMotionOnItsOwn)
{
    // Active-high, both of them: there is no inversion anywhere between the Modbus bit and this
    // struct.
    for (bool SafetyIoFlags::* const field :
         {&SafetyIoFlags::hw_e_stop_user_button, &SafetyIoFlags::sw_e_stop_latch_status})
    {
        SafetyIoFlags flags;
        flags.*field = true;
        EXPECT_TRUE(motionIsInhibited(flags));
    }
}

// Regression guard for the reason the two sw_* stops were removed. They are read-backs of coils
// this system writes, so treating one as proof the rover is safe to sweep would be a fail-open if
// the PLC never acted on the request. This node must only ever be convinced by plant state, which
// is what SafetyStatus carries; the request path reaches it solely through latch_active.
static_assert(sizeof(SafetyIoFlags) == 2 * sizeof(bool),
    "SafetyIoFlags must carry plant state only - see domain/safety_io_flags.hpp before adding a "
    "field, and never add a SafetyCommandEcho read-back here.");

TEST(SafetyIoFlagsTest, TheDefaultsAreNotEngaged)
{
    // rover_twist_mux defaults its equivalent struct to all-true, because for it "assume a stop
    // is active" denies motion and is the safe guess. Here the meaning is reversed - an engaged
    // E-Stop GRANTS permission to sweep the sticks - so the same defaults would be a silent
    // fail-open. A node that has heard nothing must not be able to build a value that says
    // "engaged".
    const SafetyIoFlags defaults;

    EXPECT_FALSE(defaults.hw_e_stop_user_button);
    EXPECT_FALSE(defaults.sw_e_stop_latch_status);
}

TEST(SafetyIoFlagsTest, TheLatchAloneIsEnough)
{
    // The latch is held after every boot, which is exactly when an operator would calibrate.
    SafetyIoFlags flags;
    flags.sw_e_stop_latch_status = true;

    EXPECT_TRUE(motionIsInhibited(flags));
}

}  // namespace
}  // namespace rover_crsf_teleop
