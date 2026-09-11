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

#ifndef ROVER_DIAG_MANAGER_DOMAIN_SYSTEM_SAMPLE_HPP_
#define ROVER_DIAG_MANAGER_DOMAIN_SYSTEM_SAMPLE_HPP_

#include <optional>
#include <vector>

namespace rover_diag_manager::domain
{

/**
 * @brief One snapshot of the host's resource usage.
 * @details A metric that could not be measured is std::nullopt, never a sentinel value.
 */
struct SystemSample
{
    std::vector<float> core_usages;          // [%] per core
    std::optional<float> cpu_mean_usage;     // [%]
    std::optional<float> cpu_temperature;    // [°C]
    std::optional<float> ram_usage;          // [%]
    std::optional<float> disk_usage;         // [%] of the root filesystem
};

}  // namespace rover_diag_manager::domain

#endif  // ROVER_DIAG_MANAGER_DOMAIN_SYSTEM_SAMPLE_HPP_
