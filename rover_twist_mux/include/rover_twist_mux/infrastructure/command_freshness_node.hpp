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

#ifndef ROVER_TWIST_MUX_INFRASTRUCTURE_COMMAND_FRESHNESS_NODE_HPP_
#define ROVER_TWIST_MUX_INFRASTRUCTURE_COMMAND_FRESHNESS_NODE_HPP_

#include <memory>
#include <string>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rover_twist_mux/command_freshness_params.hpp"
#include "rover_twist_mux/domain/command_freshness_filter.hpp"

namespace rover_twist_mux
{

/**
 * @brief Passes the Driver UI's velocity commands on to twist_mux only while they are fresh.
 * @details twist_mux judges an input by when a message arrives, not by when it was sent. The
 *          Driver UI's commands cross a websocket (TCP), which holds them through a Wi-Fi stall
 *          and then delivers them in a burst, so without this node the rover would replay
 *          seconds-old motion. The decision lives in domain::CommandFreshnessFilter; this node
 *          feeds it the sender's stamp and the local wall-clock receive time and republishes the
 *          commands it accepts, unchanged.
 *
 *          Receive time is std::chrono::system_clock, never the node clock: in simulation the
 *          node clock is sim time, which does not advance with the browser's wall-clock stamps.
 *
 *          Plain (non-lifecycle) node on purpose - it owns no hardware resource.
 */
class CommandFreshnessNode : public rclcpp::Node
{
public:
    /** @throws rclcpp::exceptions::InvalidParameterValueException on an invalid override. */
    explicit CommandFreshnessNode(
        const std::string & node_name,
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    void commandCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

    void diagnoseFreshness(diagnostic_updater::DiagnosticStatusWrapper & status);

    std::shared_ptr<command_freshness::ParamListener> param_listener_;

    std::unique_ptr<domain::CommandFreshnessFilter> filter_;

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr command_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr command_pub_;

    // Last member: its timer must not fire into a partially destroyed node.
    diagnostic_updater::Updater diagnostic_updater_;
};

}  // namespace rover_twist_mux

#endif  // ROVER_TWIST_MUX_INFRASTRUCTURE_COMMAND_FRESHNESS_NODE_HPP_
