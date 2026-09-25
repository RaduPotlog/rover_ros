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

#include "rover_crsf_teleop/domain/rate_limiter.hpp"

namespace rover_crsf_teleop
{

RateLimiter::RateLimiter(const double max_hz)
{
    if (max_hz > 0.0) {
        min_period_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(1.0 / max_hz));
    }
}

bool RateLimiter::admit(const SteadyTime now)
{
    if (last_admitted_ && now - *last_admitted_ < min_period_) {
        return false;
    }
    last_admitted_ = now;
    return true;
}

}  // namespace rover_crsf_teleop
