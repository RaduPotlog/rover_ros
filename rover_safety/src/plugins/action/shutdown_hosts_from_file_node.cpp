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


#include "rover_safety/plugins/action/shutdown_hosts_from_file_node.hpp"

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "yaml-cpp/yaml.h"

#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"

#include "rover_utils/yaml_utils.hpp"

#include "rover_safety/behavior_tree_utils.hpp"

namespace rover_safety
{

bool ShutdownHostsFromFile::updateHosts(std::vector<std::shared_ptr<infrastructure::ShutdownHostInterface>> & hosts)
{
    std::string shutdown_hosts_file;

    if (!getInput<std::string>("shutdown_hosts_file", shutdown_hosts_file) || shutdown_hosts_file.empty()) {
        RCLCPP_ERROR_STREAM(
            *this->logger_, getLoggerPrefix(name()) << "Failed to get input [shutdown_hosts_file]");

        return false;
    }

    YAML::Node shutdown_hosts;

    // Blocking file read, but bounded: updateHosts() runs once from onStart(), never per tick.

    try {
        shutdown_hosts = YAML::LoadFile(shutdown_hosts_file);
    } catch (const YAML::Exception & e) {
        RCLCPP_ERROR_STREAM(
            *this->logger_, getLoggerPrefix(name()) << "Error loading YAML file '"
            << shutdown_hosts_file << "': " << e.what());

        return false;
    }

    const YAML::Node hosts_node = shutdown_hosts.IsMap() ? shutdown_hosts["hosts"] : YAML::Node();

    if (!hosts_node || hosts_node.IsNull()) {
        return true;
    }

    if (!hosts_node.IsSequence()) {
        RCLCPP_ERROR_STREAM(
            *this->logger_, getLoggerPrefix(name()) << "'hosts' in '" << shutdown_hosts_file
            << "' must be a list.");

        return false;
    }

    try {
        for (const auto & host : hosts_node) {
            if (!host["ip"]) {
                RCLCPP_ERROR_STREAM(
                    *this->logger_, getLoggerPrefix(name()) << "Missing 'ip' for a remote host, skipping it.");
                continue;
            }

            const auto ip = rover_utils::getYAMLKeyValue<std::string>(host, "ip");
            const auto port = rover_utils::getYAMLKeyValue<std::string>(host, "port", "3003");
            const auto secret = rover_utils::getYAMLKeyValue<std::string>(host, "secret", "");
            const auto timeout = rover_utils::getYAMLKeyValue<float>(host, "timeout", 5.0f);

            hosts.push_back(std::make_shared<infrastructure::ShutdownHost>(ip, port, secret, timeout));
        }
    } catch (const std::runtime_error & e) {
        RCLCPP_ERROR_STREAM(*this->logger_, getLoggerPrefix(name()) << e.what());

        return false;
    }

    return true;
}

}  // namespace rover_safety

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<rover_safety::ShutdownHostsFromFile>("ShutdownHostsFromFile");
}
