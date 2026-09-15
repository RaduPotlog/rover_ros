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


#ifndef ROVER_SAFETY_PLUGINS_ACTION_SHUTDOWN_HOSTS_FROM_FILE_NODE_HPP_
#define ROVER_SAFETY_PLUGINS_ACTION_SHUTDOWN_HOSTS_FROM_FILE_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp/basic_types.h"

#include "rover_safety/plugins/shutdown_host.hpp"
#include "rover_safety/plugins/shutdown_hosts_node.hpp"

namespace rover_safety
{

/**
 * Asks every host listed in a YAML file to shut down (signed HTTP GET /shutdown), then waits until
 * each stops answering ping. File format:
 *
 *   hosts:
 *     - ip: 192.168.77.201   # required
 *       port: 3003           # optional, default 3003
 *       secret: <secret>     # optional, HMAC-SHA256 key
 *       timeout: 5.0         # optional, seconds to wait for the host to go down
 *
 * A file without hosts succeeds immediately; an unreadable file fails.
 */
class ShutdownHostsFromFile : public ShutdownHosts
{
public:

    ShutdownHostsFromFile(
        const std::string & name,
        const BT::NodeConfig & conf)
    : ShutdownHosts(name, conf)
    {

    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>(
                "shutdown_hosts_file", "Absolute path to a YAML file listing the hosts to shut down."),
        };
    }

private:

    bool updateHosts(std::vector<std::shared_ptr<ShutdownHostInterface>> & hosts) override;
};

}  // namespace rover_safety

#endif  // ROVER_SAFETY_PLUGINS_ACTION_SHUTDOWN_HOSTS_FROM_FILE_NODE_HPP_
