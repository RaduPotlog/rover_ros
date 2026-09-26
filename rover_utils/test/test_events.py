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

from launch import LaunchDescription, LaunchService
from launch.actions import (
    EmitEvent,
    GroupAction,
    OpaqueFunction,
    SetLaunchConfiguration,
    TimerAction,
)
from launch_ros.actions import SetParameter
import pytest

from rover_utils import events
from rover_utils.events import ControllersActive, start_once_on


class RecordingLogger:

    def __init__(self):
        self.warnings = []

    def warning(self, msg):
        self.warnings.append(msg)


@pytest.fixture
def logger(monkeypatch):
    recording = RecordingLogger()
    monkeypatch.setattr(events, '_logger', recording)
    return recording


def run_launch(entities, seen=None):
    """
    Run `entities` until launch is idle; returns how often the gated action ran.

    `seen` collects the gated action's view of the `namespace` launch configuration.
    """
    runs = []

    def gated_function(context):
        runs.append(1)
        if seen is not None:
            seen.append(context.launch_configurations.get('namespace'))
        return []

    gated = OpaqueFunction(function=gated_function)
    service = LaunchService()
    service.include_launch_description(LaunchDescription(entities(gated)))
    assert service.run(shutdown_when_idle=True) == 0
    return len(runs)


def test_event_starts_the_actions_once_and_cancels_the_fallback(logger):
    # A 60 s fallback: if the event didn't cancel it, launch would not go idle for a minute.
    runs = run_launch(lambda gated: [
        *start_once_on(ControllersActive, 60.0, [gated], 'fallback'),
        EmitEvent(event=ControllersActive()),
    ])
    assert runs == 1
    assert logger.warnings == []


def test_fallback_starts_the_actions_when_the_event_never_comes(logger):
    runs = run_launch(lambda gated: [
        *start_once_on(ControllersActive, 0.1, [gated], 'controllers not active'),
    ])
    assert runs == 1
    assert logger.warnings == ['controllers not active']


def test_an_event_after_the_fallback_does_not_start_them_again(logger):
    runs = run_launch(lambda gated: [
        *start_once_on(ControllersActive, 0.1, [gated], 'controllers not active'),
        TimerAction(period=0.3, actions=[EmitEvent(event=ControllersActive())]),
    ])
    assert runs == 1
    assert logger.warnings == ['controllers not active']


def test_other_events_are_ignored(logger):

    class OtherEvent(events.Event):
        name = 'test.OtherEvent'

    runs = run_launch(lambda gated: [
        *start_once_on(ControllersActive, 0.1, [gated], 'controllers not active'),
        EmitEvent(event=OtherEvent()),
    ])
    assert runs == 1
    assert logger.warnings == ['controllers not active']


@pytest.mark.parametrize('emit_event', [True, False])
def test_actions_see_the_configurations_from_where_they_were_placed(logger, emit_event):
    # Placed inside a scoped group, as rover_bringup does; the event arrives after the group
    # has ended.
    seen = []
    timeout = 60.0 if emit_event else 0.1
    runs = run_launch(lambda gated: [
        GroupAction(scoped=True, actions=[
            SetLaunchConfiguration('namespace', 'rover'),
            *start_once_on(ControllersActive, timeout, [gated], 'fallback'),
        ]),
        *([TimerAction(period=0.1, actions=[EmitEvent(event=ControllersActive())])]
          if emit_event else []),
    ], seen)
    assert runs == 1
    assert seen == ['rover']


@pytest.mark.parametrize('emit_event', [True, False])
def test_actions_see_launch_ros_global_parameters(logger, emit_event):
    # rover_controller.launch.py sets use_sim_time with SetParameter before rover_bringup's
    # start_once_on. launch_ros keeps that as launch configuration 'global_params', a list of
    # (name, value) tuples rather than a string; restoring it must neither fail nor drop it.
    seen = []

    def record(context):
        seen.append(list(context.launch_configurations.get('global_params', [])))
        return []

    timeout = 60.0 if emit_event else 0.1
    runs = run_launch(lambda gated: [
        GroupAction(scoped=True, actions=[
            SetParameter(name='use_sim_time', value='False'),
            *start_once_on(
                ControllersActive, timeout, [gated, OpaqueFunction(function=record)], 'fallback'),
        ]),
        *([TimerAction(period=0.1, actions=[EmitEvent(event=ControllersActive())])]
          if emit_event else []),
    ])
    assert runs == 1
    assert seen == [[('use_sim_time', 'False')]]


def test_global_parameters_set_by_the_actions_stay_inside_them(logger):
    # The started launches add their own global parameters; they must not leak into the scope
    # start_once_on was placed in.
    outer = []

    def record_outer(context):
        outer.append(list(context.launch_configurations.get('global_params', [])))
        return []

    run_launch(lambda gated: [
        SetParameter(name='use_sim_time', value='False'),
        *start_once_on(
            ControllersActive, 60.0,
            [gated, SetParameter(name='from_the_actions', value='1')], 'fallback'),
        EmitEvent(event=ControllersActive()),
        TimerAction(period=0.2, actions=[OpaqueFunction(function=record_outer)]),
    ])
    assert outer == [[('use_sim_time', 'False')]]
