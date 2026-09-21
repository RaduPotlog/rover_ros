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

#include <future>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "rover_crsf_teleop/infrastructure/rover_crsf_teleop_node.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rover_crsf_teleop::RoverCrsfTeleopNode>();

    // Single-threaded on purpose - see RoverCrsfTeleopNode. The node starts unconfigured; the
    // launch file (LifecycleNode, autostart) or a lifecycle manager drives it to active.
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());

    // On Ctrl-C, run the lifecycle shutdown transition while the context is still valid, so
    // on_shutdown() publishes the stop command before the node goes away. The callback runs on
    // rclcpp's signal thread: stop the executor and wait for spin() to return first, so the
    // transition never races controlTimerCallback() on the executor thread.
    std::promise<void> spin_exited;
    std::shared_future<void> spin_exited_future = spin_exited.get_future().share();
    auto context = node->get_node_base_interface()->get_context();
    const auto pre_shutdown_handle = context->add_pre_shutdown_callback(
        [&executor, &node, spin_exited_future]() {
            executor.cancel();
            spin_exited_future.wait();
            node->shutdown();
        });

    executor.spin();
    spin_exited.set_value();

    // No-op after Ctrl-C; otherwise this runs the callback above.
    rclcpp::shutdown();
    context->remove_pre_shutdown_callback(pre_shutdown_handle);

    return 0;
}
