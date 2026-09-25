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

import click
from launch import LaunchContext
from launch.actions import LogInfo
from launch.substitutions import TextSubstitution
from launch.utilities import perform_substitutions
import pytest

from rover_utils.messages import error_msg, flatten, warning_msg


@pytest.mark.parametrize(
    'nested, expected',
    [
        ([1, [2, [3, [4]]], 5], [1, 2, 3, 4, 5]),
        ([], []),
        ([[], [[]]], []),
        ('abc', ['abc']),  # a non-list is wrapped, and a str is not split
        ([(1, 2)], [(1, 2)]),  # only lists are flattened
    ],
)
def test_flatten(nested, expected):
    assert flatten(nested) == expected


def test_flatten_keeps_substitutions_in_order_and_leaves_the_input_alone():
    serial = TextSubstitution(text='A1-2026-01')
    version = TextSubstitution(text='1.0')
    nested = ['Serial Number: ', serial, '\n', ['Robot Version: ', [version], '\n']]

    flat = flatten(nested)

    assert flat == ['Serial Number: ', serial, '\n', 'Robot Version: ', version, '\n']
    assert flat[1] is serial
    assert flat[4] is version
    assert nested[3] == ['Robot Version: ', [version], '\n']


# Only the text, colour and bold are checked: the exact escape string depends on the click version.
@pytest.mark.parametrize('make_msg, color', [(error_msg, '\x1b[31m'), (warning_msg, '\x1b[33m')])
def test_error_and_warning_msg_log_the_text_bold_in_red_or_yellow(make_msg, color):
    action = make_msg('Unsupported OS version')
    assert isinstance(action, LogInfo)

    logged = perform_substitutions(LaunchContext(), action.msg)

    assert click.unstyle(logged) == 'Unsupported OS version'
    assert color in logged
    assert '\x1b[1m' in logged
