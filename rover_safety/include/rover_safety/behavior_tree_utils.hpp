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

#ifndef ROVER_SAFETY_BEHAVIOR_TREE_UTILS_HPP_
#define ROVER_SAFETY_BEHAVIOR_TREE_UTILS_HPP_

#include <any>
#include <chrono>
#include <map>
#include <stdexcept>
#include <string>
#include <typeinfo>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/tree_node.h"
#include "behaviortree_cpp/utils/shared_library.h"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace rover_safety
{

inline std::string getLoggerPrefix(const std::string & bt_node_name)
{
    return std::string("[" + bt_node_name + "] ");
}

inline bool timeoutExceeded(
    const std::chrono::time_point<std::chrono::steady_clock> & start_time,
    const std::chrono::milliseconds & timeout)
{
    return std::chrono::steady_clock::now() - start_time > timeout;
}

}  // namespace rover_safety

#endif  // ROVER_SAFETY_BEHAVIOR_TREE_UTILS_HPP_
