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

// rclcpp_components registration of RoverBatteryNode. The launch file loads it into one
// container with rover_udp_driver's UdpReceiverNode, so BMS packets reach it intra-process
// instead of through the Zenoh router.
//
// Compiled straight into the SHARED rover_battery_component library, never into the STATIC
// _ros archive: the linker drops archive members nothing references, and with them the static
// registration.

#include <rclcpp_components/register_node_macro.hpp>

#include "rover_battery/infrastructure/rover_battery_node.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(rover_battery::RoverBatteryNode)
