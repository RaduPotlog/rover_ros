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

from launch.some_substitutions_type import SomeSubstitutionsType
from launch.substitutions import PythonExpression


def limit_log_level_to_info(unit: SomeSubstitutionsType, log_level: SomeSubstitutionsType):
    """
    Return a `--log-level` value for `unit` that follows `log_level` but never goes below INFO.

    The comparison has to happen inside the PythonExpression: `log_level` is only known when
    launch performs the substitution, so a Python `if` here would test the Substitution object
    itself (always truthy).
    """
    return PythonExpression(
        [
            "'",
            unit,
            ":=' + ('INFO' if '",
            log_level,
            "'.upper() == 'DEBUG' else '",
            log_level,
            "'.upper())",
        ]
    )

def quiet_rmw_zenoh(log_level: SomeSubstitutionsType):
    """
    Return a `--log-level` value capping rmw_zenoh_cpp at ERROR.

    For nodes we don't own: on Ctrl-C rmw_zenoh closes its session ~2 s before the context is
    invalidated, and their timers log "unable to publish message since the zenoh session is
    closed" on every publish in between. DEBUG (and ERROR/FATAL) runs keep the chosen level.
    """
    return PythonExpression(
        [
            "'rmw_zenoh_cpp:=' + ('",
            log_level,
            "' if '",
            log_level,
            "'.upper() in ('DEBUG', 'ERROR', 'FATAL') else 'ERROR')",
        ]
    )
