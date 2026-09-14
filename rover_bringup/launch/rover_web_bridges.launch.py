#!/usr/bin/env python3

# Copyright 2025 Mechatronics Academy
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Web bridges (foxglove_bridge, rosbridge) under rover_-prefixed node names."""

from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # The upstream launch file keeps every foxglove_bridge parameter default. Its <node>
    # has no name, so a global __node remap is the only node-name rule and renames it.
    # (It would not work on a named node: launch_ros puts `-r __node:=<name>` first and rcl
    # uses the first matching rule.)
    foxglove_bridge = GroupAction(
        scoped=True,
        actions=[
            SetRemap(src="__node", dst="rover_foxglove_bridge"),
            IncludeLaunchDescription(
                FrontendLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("foxglove_bridge"), "launch", "foxglove_bridge_launch.xml"]
                    )
                )
            ),
        ],
    )

    # rosbridge_websocket_launch.xml names its nodes, so start them directly; both declare
    # the same parameter defaults the upstream launch file passes (port 9090).
    rosbridge_websocket = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        name="rover_rosbridge_websocket",
        output="screen",
    )

    # Kept as /rosapi: rosbridge clients (ros-mcp-server) call the /rosapi/* services.
    rosapi = Node(
        package="rosapi",
        executable="rosapi_node",
        name="rosapi",
        output="screen",
    )

    return LaunchDescription([foxglove_bridge, rosbridge_websocket, rosapi])
