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

// rclcpp_components registration of SerialBridgeNode, so a launch file can load it into the
// component container of the node it serves (rover_crsf_teleop) and hand over its bytes
// intra-process instead of through the Zenoh router.
//
// Compiled straight into the SHARED rover_serial_driver_components library, never into the
// STATIC _ros archive: the linker drops archive members nothing references, and with them the
// static registration. rover_serial_bridge_node keeps its own main, which runs the lifecycle
// shutdown transition on Ctrl-C; rclcpp_components' generated main does not.

#include <rclcpp_components/register_node_macro.hpp>

#include "rover_serial_driver/infrastructure/serial_bridge_node.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(rover::transport::serial::SerialBridgeNode)
