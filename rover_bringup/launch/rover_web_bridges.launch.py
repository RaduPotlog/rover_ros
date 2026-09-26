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

import os

from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare


# Upstream's default allowlist spells the extensions lowercase and compiles the pattern with
# a case-sensitive std::regex, so a mesh named `.STL` is refused and its link renders empty.
# std::regex uses the ECMAScript grammar, which rejects the inline `(?i)` flag (it throws
# "Invalid '(?...)' zero-width assertion"), so each letter becomes a two-case character class.
# Extension list is upstream's, verbatim.
_ASSET_EXTENSIONS = (
    "dae fbx glb gltf jpeg jpg mtl obj png stl tif tiff urdf webp xacro".split()
)
_ANY_CASE = "|".join(
    "".join(f"[{c}{c.upper()}]" for c in ext) for ext in _ASSET_EXTENSIONS
)
FOXGLOVE_ASSET_URI_ALLOWLIST = (
    r"['^package://(?:[-\w%]+/)*[-\w%.]+\." + f"(?:{_ANY_CASE})" + r"$']"
)

# Topics the web UIs actually use (rover_drive_interface through nginx's /ws, the Cockpit
# plugin through cockpit-bridge). Every other topic stays off the bridge: upstream's
# default ['.*'] advertises the whole graph, and anything a browser (or a stray Foxglove Studio)
# subscribes to crosses the Zenoh router into this process at its full rate - the UIs throttle
# only their redraws, never what the bridge sends. Names are relative to the rover namespace,
# matched under any namespace. When a UI starts using a new topic, add it here - including
# topics it only PUBLISHES: the UIs' foxglove client builds a publisher's message encoder from
# the schema of the server channel with the same name, and waits for that channel forever if the
# bridge never advertises it (manual driving silently sent nothing on 2026-09-26).
# ROVER_FOXGLOVE_TOPIC_WHITELIST="['.*']" on the platform service opens it up for debugging.
_ABSOLUTE_UI_TOPICS = ("/tf", "/tf_static")
_NAMESPACED_UI_TOPICS = (
    # rover_drive_interface: map view
    "map",
    "global_costmap/costmap",
    "scan",
    "plan",
    # rover_drive_interface: status, safety, localization and missions
    "hardware_interface/aux_io_state",
    "hardware_interface/safety_status",
    "hardware_interface/safety_command_echo",
    "motion_lock",
    "rover_battery/battery_status",
    "rover_battery/charging_status",
    "localization_state",
    "maps",
    "places",
    "amcl_pose",
    "mission_state",
    # rover_drive_interface: published by the UI (see above for why they must be listed)
    "teleop_driver_interface_cmd_vel_stamped",
    "initialpose",
    # both UIs
    "diagnostics_agg",
    # Cockpit: LED page (led/channel_<n>_preview, not the 50 Hz _frame the driver consumes)
    "led/animations",
    "led/state",
    "led/brightness",
    r"led/channel_\d+_preview",
    # Cockpit: RC page
    "rc/channels",
    "rc/link",
    "rc/calibration/state",
)
FOXGLOVE_TOPIC_WHITELIST = (
    "["
    + ",".join(
        [f"'^{topic}$'" for topic in _ABSOLUTE_UI_TOPICS]
        + [rf"'^(?:/\w+)*/{topic}$'" for topic in _NAMESPACED_UI_TOPICS]
    )
    + "]"
)


def generate_launch_description():
    # The upstream launch file keeps every foxglove_bridge parameter default except
    # asset_uri_allowlist, topic_whitelist and sysinfo, overridden below. Its <node> has no
    # name, so a global __node remap is the only node-name rule and renames it. (It would not
    # work on a named node: launch_ros puts `-r __node:=<name>` first and rcl uses the first
    # matching rule.) Passing a launch *argument* does not name the node, so the remap still
    # applies.
    foxglove_bridge = GroupAction(
        scoped=True,
        actions=[
            SetRemap(src="__node", dst="rover_foxglove_bridge"),
            IncludeLaunchDescription(
                FrontendLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("foxglove_bridge"), "launch", "foxglove_bridge_launch.xml"]
                    )
                ),
                launch_arguments={
                    "asset_uri_allowlist": FOXGLOVE_ASSET_URI_ALLOWLIST,
                    # Empty counts as unset: docker-compose.yml declares the variable blank.
                    "topic_whitelist": os.environ.get("ROVER_FOXGLOVE_TOPIC_WHITELIST")
                    or FOXGLOVE_TOPIC_WHITELIST,
                    # No UI reads /foxglove_bridge/sysinfo, which otherwise publishes every 500 ms.
                    "sysinfo": "false",
                }.items(),
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
