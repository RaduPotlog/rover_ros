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

#include "rover_crsf_teleop/domain/rate_limiter.hpp"

namespace rover_crsf_teleop
{
namespace
{

using std::chrono::milliseconds;

const SteadyTime kStart{};

}  // namespace

TEST(RateLimiterTest, FirstEventPasses)
{
    RateLimiter limiter(25.0);
    EXPECT_TRUE(limiter.admit(kStart));
}

TEST(RateLimiterTest, DropsEventsInsideThePeriod)
{
    RateLimiter limiter(25.0);  // 40 ms

    ASSERT_TRUE(limiter.admit(kStart));
    EXPECT_FALSE(limiter.admit(kStart + milliseconds(4)));
    EXPECT_FALSE(limiter.admit(kStart + milliseconds(39)));
    EXPECT_TRUE(limiter.admit(kStart + milliseconds(40)));
    EXPECT_FALSE(limiter.admit(kStart + milliseconds(44)));
}

TEST(RateLimiterTest, CapsAFastStreamAtTheConfiguredRate)
{
    RateLimiter limiter(25.0);

    int admitted = 0;
    for (int ms = 0; ms < 1000; ms += 4) {  // a 250 Hz receiver for one second
        admitted += limiter.admit(kStart + milliseconds(ms)) ? 1 : 0;
    }
    EXPECT_EQ(admitted, 25);
}

TEST(RateLimiterTest, AfterAGapTheNextEventPassesAtOnce)
{
    RateLimiter limiter(25.0);

    ASSERT_TRUE(limiter.admit(kStart));
    EXPECT_TRUE(limiter.admit(kStart + milliseconds(500)));
    EXPECT_FALSE(limiter.admit(kStart + milliseconds(501)));
}

TEST(RateLimiterTest, ZeroOrNegativeRateLetsEverythingThrough)
{
    for (const double hz : {0.0, -1.0}) {
        RateLimiter limiter(hz);
        EXPECT_TRUE(limiter.admit(kStart));
        EXPECT_TRUE(limiter.admit(kStart));
        EXPECT_TRUE(limiter.admit(kStart + milliseconds(1)));
    }
}

TEST(RateLimiterTest, ResetLetsTheNextEventThrough)
{
    RateLimiter limiter(1.0);

    ASSERT_TRUE(limiter.admit(kStart));
    EXPECT_FALSE(limiter.admit(kStart + milliseconds(10)));
    limiter.reset();
    EXPECT_TRUE(limiter.admit(kStart + milliseconds(20)));
}

}  // namespace rover_crsf_teleop
