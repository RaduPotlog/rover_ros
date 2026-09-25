// Copyright 2026 Mechatronics Academy
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

#ifndef ROVER_UTILS_ROVER_UTILS_PARAMETER_UTILS_HPP_
#define ROVER_UTILS_ROVER_UTILS_PARAMETER_UTILS_HPP_

#include <string>

#include "rcl_interfaces/msg/parameter_descriptor.hpp"

namespace rover_utils::ros
{

/** @brief Descriptor of a read-only parameter with the given description. */
inline rcl_interfaces::msg::ParameterDescriptor describe(const std::string & description)
{
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = description;
    descriptor.read_only = true;
    return descriptor;
}

/** @brief describe(), limited to [1e-6, max_value]: a read-only, strictly positive double. */
inline rcl_interfaces::msg::ParameterDescriptor describePositive(
    const std::string & description, double max_value)
{
    auto descriptor = describe(description);
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = 1.0e-6;
    descriptor.floating_point_range[0].to_value = max_value;
    return descriptor;
}

}  // namespace rover_utils::ros

#endif  // ROVER_UTILS_ROVER_UTILS_PARAMETER_UTILS_HPP_
