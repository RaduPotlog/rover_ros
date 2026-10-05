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

#include "RosContextShutdown.hh"

#include <atomic>

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <rclcpp/rclcpp.hpp>

namespace rover_gazebo_plugins
{

namespace
{

// Live instances; the last one destroyed shuts the context down.
std::atomic<int> & instanceCount()
{
  static std::atomic<int> count{0};
  return count;
}

}  // namespace

RosContextShutdown::RosContextShutdown()
{
  ++instanceCount();
}

RosContextShutdown::~RosContextShutdown()
{
  if (--instanceCount() > 0) {
    return;
  }
  if (rclcpp::ok()) {
    gzmsg << "[RosContextShutdown] Shutting down the ROS context before the server exits.\n";
    rclcpp::shutdown(nullptr, "gz-sim server shutting down");
  }
}

void RosContextShutdown::Configure(
  const gz::sim::Entity &, const std::shared_ptr<const sdf::Element> &,
  gz::sim::EntityComponentManager &, gz::sim::EventManager &)
{
}

}  // namespace rover_gazebo_plugins

GZ_ADD_PLUGIN(
  rover_gazebo_plugins::RosContextShutdown, gz::sim::System,
  gz::sim::ISystemConfigure)
