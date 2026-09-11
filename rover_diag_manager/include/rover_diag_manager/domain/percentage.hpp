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

#ifndef ROVER_DIAG_MANAGER_DOMAIN_PERCENTAGE_HPP_
#define ROVER_DIAG_MANAGER_DOMAIN_PERCENTAGE_HPP_

#include <optional>
#include <vector>

namespace rover_diag_manager::domain
{

/** @brief Rounds `value` to `decimals` decimal places (half away from zero). */
float roundTo(float value, unsigned int decimals);

/**
 * @brief `part` as a percentage of `total`, rounded to 2 decimals.
 * @return std::nullopt if `total` is not positive.
 */
std::optional<float> percentageOf(double part, double total);

/**
 * @brief Mean of per-core usages, rounded to 2 decimals.
 * @return std::nullopt if `core_usages` is empty or any value is outside [0, 100].
 */
std::optional<float> meanUsage(const std::vector<float> & core_usages);

}  // namespace rover_diag_manager::domain

#endif  // ROVER_DIAG_MANAGER_DOMAIN_PERCENTAGE_HPP_
