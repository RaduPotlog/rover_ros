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
#include <sstream>
#include <stdexcept>
#include <string>
#include <typeinfo>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/tree_node.h"
#include "behaviortree_cpp/utils/shared_library.h"


namespace rover_safety
{

/**
 * Registers the BT nodes of every plugin library, then the trees of a BehaviorTree project.
 *
 * Library names are given without prefix/suffix (e.g. `call_trigger_service_bt_node`) and resolved
 * through the library search path. ROS plugins (`ros_plugin_libs`) are nav2_behavior_tree nodes that
 * take the ROS node from the blackboard entry `node`, so both lists load the same way; they are kept
 * apart to match the parameter files. Throws BT::RuntimeError when a library or the project cannot
 * be loaded.
 */
inline void registerBehaviorTree(
    BT::BehaviorTreeFactory & factory,
    const std::string & bt_project_path,
    const std::vector<std::string> & plugin_libs,
    const std::vector<std::string> & ros_plugin_libs)
{
    for (const auto & plugin : ros_plugin_libs) {
        factory.registerFromPlugin(BT::SharedLibrary::getOSName(plugin));
    }

    for (const auto & plugin : plugin_libs) {
        factory.registerFromPlugin(BT::SharedLibrary::getOSName(plugin));
    }

    factory.registerBehaviorTreeFromFile(bt_project_path);
}

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
