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

from launch.actions import Shutdown


def shutdown_unless_shutting_down(process_name: str):
    """`on_exit` handler for a required process.

    Shuts the launch down when the process exits on its own, but emits nothing when launch is
    already shutting down (e.g. Ctrl-C). A second Shutdown during teardown makes launch_ros run
    its ROS adapter shutdown twice: "Cannot shutdown a ROS adapter that is not running".
    """
    def on_exit(event, context):
        if context.is_shutdown:
            return []
        return [Shutdown(reason=f'{process_name} exited with code {event.returncode}')]

    return on_exit
