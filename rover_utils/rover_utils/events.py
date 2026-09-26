#!/usr/bin/env python3

# Copyright 2026 Mechatronics Academy
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

import copy
from typing import List, Type

from launch import Action, Event, EventHandler
from launch.actions import (
    OpaqueFunction,
    PopEnvironment,
    PopLaunchConfigurations,
    PushEnvironment,
    PushLaunchConfigurations,
    RegisterEventHandler,
    ReplaceEnvironmentVariables,
    ResetLaunchConfigurations,
    TimerAction,
)
import launch.logging
from launch.some_substitutions_type import SomeSubstitutionsType

_logger = launch.logging.get_logger('rover_utils.events')


class ControllersActive(Event):
    """Emitted by rover_controller.launch.py once every mandatory controller is active."""

    name = 'rover_utils.events.ControllersActive'


def start_once_on(
    event_type: Type[Event],
    timeout: SomeSubstitutionsType,
    actions: List[Action],
    fallback_msg: str,
) -> List[Action]:
    """
    Start `actions` when an `event_type` event arrives, or after `timeout` seconds at the latest.

    Whichever comes first starts them, exactly once. The timeout is a fallback for an event that
    never arrives (e.g. a hung spawner); it logs `fallback_msg` as a warning when it fires.

    Like TimerAction, the actions see the launch configurations and environment from where this
    was placed, not whatever is current when the event arrives (e.g. after an enclosing scoped
    group has ended), and they are restored the same way TimerAction restores them.
    """
    started = False
    configurations = {}
    environment = {}

    def snapshot(context):
        # Copies of the values: launch_ros's SetParameter extends the 'global_params' list in
        # place, so a shared list would let later changes leak in either direction.
        configurations.update(
            {name: copy.copy(value) for name, value in context.launch_configurations.items()})
        environment.update(context.environment)
        return []

    def start(context, fallback=False):
        nonlocal started
        if started:
            return []
        started = True
        if fallback:
            # TimerAction has already restored its own snapshot of the configurations.
            _logger.warning(fallback_msg)
            return actions
        timer.cancel()
        # As TimerAction.handle() does it. Not SetLaunchConfiguration per entry: launch_ros keeps
        # non-string configurations, such as 'global_params' (a list of (name, value) tuples),
        # which SetLaunchConfiguration can't take; ResetLaunchConfigurations keeps them as they
        # are.
        return [
            PushEnvironment(),
            PushLaunchConfigurations(),
            ReplaceEnvironmentVariables(environment),
            ResetLaunchConfigurations(
                {name: copy.copy(value) for name, value in configurations.items()}),
            *actions,
            PopEnvironment(),
            PopLaunchConfigurations(),
        ]

    timer = TimerAction(
        period=timeout,
        actions=[OpaqueFunction(function=start, kwargs={'fallback': True})],
    )

    return [
        OpaqueFunction(function=snapshot),
        RegisterEventHandler(
            EventHandler(
                matcher=lambda event: isinstance(event, event_type),
                entities=[OpaqueFunction(function=start)],
                handle_once=True,
            )
        ),
        timer,
    ]
