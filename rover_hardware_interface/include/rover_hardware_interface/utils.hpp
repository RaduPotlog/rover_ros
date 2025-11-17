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

#ifndef ROVER_HARDWARE_INTERFACE_UTILS_HPP_
#define ROVER_HARDWARE_INTERFACE_UTILS_HPP_

#include <cstdint>
#include <functional>
#include <stdexcept>
#include <string>

namespace rover_hardware_interface
{

bool operationWithAttempts(
    const std::function<void()> operation, 
    const unsigned max_attempts,
    const std::function<void()> on_error = []() {});

bool checkIfJointNameContainValidSequence(
    const std::string & name, 
    const std::string & sequence);

}  // namespace rover_hardware_interface

#endif  // ROVER_HARDWARE_INTERFACE_UTILS_HPP_
