# Copyright 2026 Rover A1 contributors
# Licensed under the Apache License, Version 2.0.

import pytest

from rover_controller.schedule import active_segment, build_schedule, Segment


def test_build_schedule_rests_before_every_command_and_at_the_end():
    schedule = build_schedule([(0.5, 0.0, 'fwd'), (0.0, 1.0, 'spin')], hold_time=2.0,
                              rest_time=1.0)
    assert schedule == [
        Segment(0.0, 1.0, 0.0, 0.0, 'rest'),
        Segment(1.0, 3.0, 0.5, 0.0, 'fwd'),
        Segment(3.0, 4.0, 0.0, 0.0, 'rest'),
        Segment(4.0, 6.0, 0.0, 1.0, 'spin'),
        Segment(6.0, 7.0, 0.0, 0.0, 'rest'),
    ]


def test_build_schedule_without_commands_is_one_rest():
    assert build_schedule([], hold_time=2.0, rest_time=1.5) == [
        Segment(0.0, 1.5, 0.0, 0.0, 'rest')]


@pytest.mark.parametrize('t, label', [(0.0, 'rest'), (0.99, 'rest'), (1.0, 'fwd'),
                                      (2.99, 'fwd'), (3.0, 'rest')])
def test_active_segment_is_start_inclusive_end_exclusive(t, label):
    schedule = build_schedule([(0.5, 0.0, 'fwd')], hold_time=2.0, rest_time=1.0)
    assert active_segment(schedule, t).label == label


def test_active_segment_after_the_schedule_is_none():
    schedule = build_schedule([(0.5, 0.0, 'fwd')], hold_time=2.0, rest_time=1.0)
    assert active_segment(schedule, 4.0) is None
