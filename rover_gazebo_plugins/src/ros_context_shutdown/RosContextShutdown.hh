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

#ifndef ROVER_GAZEBO_PLUGINS__ROS_CONTEXT_SHUTDOWN_HH_
#define ROVER_GAZEBO_PLUGINS__ROS_CONTEXT_SHUTDOWN_HH_

#include <memory>

#include <gz/sim/System.hh>

namespace rover_gazebo_plugins
{

// Shuts the default rclcpp context down while the gz-sim server is torn down.
//
// gz_ros2_control calls rclcpp::init() in the server process but never rclcpp::shutdown(), so
// the context is shut down by its static destructor inside exit(). Under rmw_zenoh_cpp that
// runs after Zenoh's Tokio thread-local storage is gone and the process aborts ("The Thread
// Local Storage inside Tokio is destroyed"); the gazebo launch action is required, so the whole
// launch then goes down with an error. See rmw_zenoh's "Known issues".
//
// Load it in the same <gazebo> block, after gz_ros2_control: gz-sim destroys systems in load
// order, so the controller manager has stopped before the context goes. With several models
// each instance holds a reference and the last one destroyed shuts the context down.
class RosContextShutdown : public gz::sim::System, public gz::sim::ISystemConfigure
{
public:
  RosContextShutdown();
  ~RosContextShutdown() override;

  void Configure(
    const gz::sim::Entity & entity, const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager & ecm, gz::sim::EventManager & event_manager) override;
};

}  // namespace rover_gazebo_plugins

#endif  // ROVER_GAZEBO_PLUGINS__ROS_CONTEXT_SHUTDOWN_HH_
