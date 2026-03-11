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

import textwrap
from typing import Dict

import click
from launch.actions import LogInfo
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.substitutions import Command, PythonExpression

ROVER_A1_ASCII = r"""
     ____   ____  _        _  ____  ____        _         
    |    | |    |  \      /  |     |    |      / \       /|
    |____| |    |   \    /   |___  |____| --  /___\     / |
    |  \   |    |    \  /    |     |  \      /     \      |
    |   \_ |____|     \/     |____ |   \_  _/       \_  __|__
           
    """

ROVER_A1_TEXT = click.style(textwrap.dedent(ROVER_A1_ASCII), bold=True)

class ErrorMessages:
    INCORRECT_HW_CONFIG = textwrap.dedent(
        r"""

        ERROR: Incorrect hardware configuration detected. ROS nodes are prevented from starting!
        """
    )

    INCORRECT_OS_VERSION = textwrap.dedent(
        r"""

        WARNING: Unsupported OS version detected. ROS diver may not work correctly.
        Please update your system to the latest version.
        """
    )


def flatten(lst):
    """Flatten a nested list into a single list."""
    if isinstance(lst, list):
        flat_list = []
        for element in lst:
            flat_list.extend(flatten(element))
        return flat_list
    else:
        return [lst]


def welcome_msg(
    robot_model: SomeSubstitutionsType,
    serial_number: SomeSubstitutionsType,
    robot_hw_version: SomeSubstitutionsType,
    additional_stats: Dict = {},
):
    """Generate a welcome message with robot information and stats."""
    pkg_version = Command(command="ros2 pkg xml -t version rover_metapackage")

    robot_model_expr = PythonExpression(
        [f"r'''{ROVER_A1_TEXT}''' if '", robot_model, f"' == 'rover_a1' else r'''{""}'''"]
    )

    stats_to_show = {
        "Serial Number": serial_number,
        "Robot Version": robot_hw_version,
        "ROS Driver Version": pkg_version,
        **additional_stats,
        "Website": "https://mechatronocs-academy.com",
        "Support": "https://community.mechatronocs-academy.com/",
        "Bug Tracker": "https://github.com/RaduPotlog/rover_ros",
    }

    nested_list_of_stats = [
        item
        for name, value in stats_to_show.items()
        for item in (f"{click.style(name, bold=True)}: ", value, "\n")
    ]
    stats_msg = flatten(nested_list_of_stats)

    stats_msg.insert(0, robot_model_expr)

    return LogInfo(msg=stats_msg)


def error_msg(error: str):
    """Generate an error message."""
    return LogInfo(msg=click.style(error, bold=True, fg="red"))


def warning_msg(warning: str):
    """Generate a warning message."""
    return LogInfo(msg=click.style(warning, bold=True, fg="yellow"))
