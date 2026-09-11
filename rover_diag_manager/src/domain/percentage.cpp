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

#include "rover_diag_manager/domain/percentage.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <optional>
#include <vector>

namespace rover_diag_manager::domain
{

namespace
{
constexpr unsigned int kPercentageDecimals = 2;
}  // namespace

float roundTo(float value, unsigned int decimals)
{
    const double scale = std::pow(10.0, decimals);
    return static_cast<float>(std::round(static_cast<double>(value) * scale) / scale);
}

std::optional<float> percentageOf(double part, double total)
{
    if (!(total > 0.0)) {
        return std::nullopt;
    }

    return roundTo(static_cast<float>(part / total * 100.0), kPercentageDecimals);
}

std::optional<float> meanUsage(const std::vector<float> & core_usages)
{
    if (core_usages.empty()) {
        return std::nullopt;
    }

    // Written as a negated in-range check so NaN is rejected as well.
    const bool all_valid = std::all_of(
        core_usages.begin(), core_usages.end(),
        [](float usage) {return usage >= 0.0f && usage <= 100.0f;});
    if (!all_valid) {
        return std::nullopt;
    }

    const double sum = std::accumulate(core_usages.begin(), core_usages.end(), 0.0);
    return roundTo(
        static_cast<float>(sum / static_cast<double>(core_usages.size())), kPercentageDecimals);
}

}  // namespace rover_diag_manager::domain
