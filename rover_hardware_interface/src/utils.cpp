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

#include "rover_hardware_interface/utils.hpp"

#include <iostream>
#include <stdexcept>
#include <thread>

namespace rover_hardware_interface
{

// std::cerr (not RCLCPP_*) is deliberate here: utils.hpp is included by
// domain/driver_data_snapshot.hpp (for DrivetrainSettings) and must stay transitively free of
// rclcpp, or the domain layer's ROS-independence (enforced by scripts/check_domain_purity.sh for
// direct includes) would be silently broken one include deeper than that script can check.

bool operationWithAttempts(
    const std::function<void()> operation,
    const unsigned max_attempts,
    const std::function<void()> on_error,
    const std::chrono::milliseconds delay_between_attempts)
{
    for (unsigned i = 0; i < max_attempts; ++i) {
        try {
            operation();
            return true;
        } catch (const std::runtime_error & e) {
            std::cerr << "An exception occurred while handling operation() function, attempt " << i + 1
                      << " of " << max_attempts << ": " << e.what() << std::endl;
            try {
                on_error();
            } catch (const std::runtime_error & on_error_e) {
                std::cerr << "An exception occurred while handling on_error() function: "
                        << on_error_e.what() << std::endl;
                return false;
            }

            if (delay_between_attempts.count() > 0 && i + 1 < max_attempts) {
                std::this_thread::sleep_for(delay_between_attempts);
            }
        }
    }

    return false;
}

bool checkIfJointNameContainValidSequence(
    const std::string & name,
    const std::string & sequence)
{
    // `sequence` (e.g. "fl") must be the leading token of the joint's own local name — i.e.
    // right after any namespace prefix (everything up to and including the last '/'), and
    // either the whole local name or immediately followed by '_'. This is deliberately
    // stricter than "sequence appears anywhere as a delimited substring": that older check
    // could false-positive on a namespace segment that happens to equal `sequence` (e.g. a
    // "my_fr_robot/fl_..." namespace containing "fr" as its own delimited token), which a
    // plain substring-with-delimiter-boundaries scan can't distinguish from the real prefix.
    const std::size_t namespace_end = name.find_last_of('/');
    const std::size_t local_name_start = (namespace_end == std::string::npos) ? 0 : namespace_end + 1;

    if (name.compare(local_name_start, sequence.length(), sequence) != 0) {
        return false;
    }

    const std::size_t sequence_end = local_name_start + sequence.length();

    return sequence_end == name.length() || name[sequence_end] == '_';
}

}  // namespace rover_hardware_interface