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

#ifndef ROVER_CRSF_TELEOP_DOMAIN_RATE_LIMITER_HPP_
#define ROVER_CRSF_TELEOP_DOMAIN_RATE_LIMITER_HPP_

#include <chrono>
#include <optional>

#include "rover_crsf_teleop/domain/link_monitor.hpp"

namespace rover_crsf_teleop
{

// Lets through at most `max_hz` events per second; `max_hz` <= 0 lets every event through.
//
// Used for the rc/channels and rc/link echoes. The receiver decodes a frame every ~4 ms, and each
// echo crosses the Zenoh router to the web UI, which refreshes at 10 Hz. The teleop rules, the
// diagnostics and the calibration still see every frame: only the echo is limited.
//
// The next slot is scheduled from the admitted event, not from a fixed grid, and an event that
// arrives after a gap is admitted at once - there is no burst to catch up.
class RateLimiter
{

public:

    explicit RateLimiter(double max_hz);

    // True when an event at `now` may pass; records it as the last admitted event.
    bool admit(SteadyTime now);

    // Forgets the last admitted event, so the next one passes.
    void reset() { last_admitted_.reset(); }

private:

    std::chrono::steady_clock::duration min_period_{};
    std::optional<SteadyTime> last_admitted_;
};

}  // namespace rover_crsf_teleop

#endif  // ROVER_CRSF_TELEOP_DOMAIN_RATE_LIMITER_HPP_
