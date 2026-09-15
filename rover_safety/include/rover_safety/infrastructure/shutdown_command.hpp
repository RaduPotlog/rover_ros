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

#ifndef ROVER_SAFETY_INFRASTRUCTURE_SHUTDOWN_COMMAND_HPP_
#define ROVER_SAFETY_INFRASTRUCTURE_SHUTDOWN_COMMAND_HPP_

#include <string>

namespace rover_safety::infrastructure
{

/** Environment variable carrying the shutdown reason; read by shutdown_ros_controller.sh. */
inline constexpr char kShutdownReasonEnv[] = "ROVER_SHUTDOWN_REASON";

/** Quotes `value` as one single-quoted bash word, e.g. it's -> 'it'\''s'. */
std::string shellQuote(const std::string & value);

/**
 * The bash command the shutdown tree's ExecuteCommand runs: exports the reason as
 * ROVER_SHUTDOWN_REASON, then runs `command` verbatim.
 */
std::string buildShutdownCommand(const std::string & command, const std::string & reason);

}  // namespace rover_safety::infrastructure

#endif  // ROVER_SAFETY_INFRASTRUCTURE_SHUTDOWN_COMMAND_HPP_
