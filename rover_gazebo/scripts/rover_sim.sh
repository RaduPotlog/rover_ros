#!/usr/bin/env bash

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

# Runs the Rover A1 Gazebo simulation from a fresh terminal, whatever the login shell set up
# for the real rover:
#   - ROVER_ROS_BUILD_TYPE=simulation (~/.bashrc exports "hardware").
#   - rmw_zenoh on a local router bound to localhost. ZENOH_CONFIG_OVERRIDE from the login
#     shell (client mode to the rover's router) is dropped, so the sim never joins the rover.
#     FastDDS is not used: inter-process discovery does not work on this WSL host.
#
# Usage:
#   rover_sim.sh [--build] [launch args...]   start the simulation (Ctrl+C stops everything)
#   source rover_sim.sh                       only set up this shell, e.g. a second terminal
#                                             for rover_navigation or ros2 CLI tools
#
# Examples:
#   rover_sim.sh --build
#   ROVER_USE_GPS=true rover_sim.sh use_rviz:=False gz_headless_mode:=True
#   source rover_sim.sh && ros2 launch rover_navigation bringup.launch.py \
#     use_sim_time:=True localization_source:=slam
#
# Environment (all optional):
#   ROVER_WS          workspace root (default: found from this script, else ~/ros2_ws/rover_a1)
#   ROVER_NAMESPACE   robot namespace (default: rover, as in docker-compose)
#   ROVER_USE_GPS, ROVER_GPS_PUBLISH_MAP_TF   passed through to the simulation

_rover_sim_is_sourced() { [ "${BASH_SOURCE[0]}" != "$0" ]; }

_rover_sim_find_ws() {
    if [ -n "${ROVER_WS:-}" ]; then
        echo "$ROVER_WS"
        return
    fi
    # With --symlink-install the installed script links back here, so walk up from the source.
    local dir
    dir=$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")
    while [ "$dir" != "/" ]; do
        if [ -d "$dir/src" ] && [ -d "$dir/src/rover_ros" ]; then
            echo "$dir"
            return
        fi
        dir=$(dirname "$dir")
    done
    echo "$HOME/ros2_ws/rover_a1"
}

_rover_sim_setup_env() {
    ROVER_WS=$(_rover_sim_find_ws)
    export ROVER_WS

    export ROVER_ROS_BUILD_TYPE=simulation
    export ROVER_NAMESPACE="${ROVER_NAMESPACE:-rover}"

    unset ZENOH_CONFIG_OVERRIDE ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS
    export RMW_IMPLEMENTATION=rmw_zenoh_cpp

    # A stale overlay (e.g. a hardware install sourced by ~/.bashrc) would shadow this build.
    unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH

    local distro
    distro=$(ls /opt/ros | head -n 1)
    # ROS setup scripts are not nounset-safe.
    # shellcheck disable=SC1090
    source "/opt/ros/${distro}/setup.bash"

    if [ -f "$ROVER_WS/install/setup.bash" ]; then
        # shellcheck disable=SC1091
        source "$ROVER_WS/install/setup.bash"
    fi

    # A ros2 CLI daemon started from the login shell keeps the rover's zenoh config, and the
    # CLI would then list the rover's graph instead of the sim's. It restarts on demand.
    ros2 daemon stop > /dev/null 2>&1 || true
}

_rover_sim_router_running() {
    ss -Hltn 'sport = :7447' 2>/dev/null | grep -q LISTEN
}

if _rover_sim_is_sourced; then
    _rover_sim_setup_env
    if ! _rover_sim_router_running; then
        echo "rover_sim: no zenoh router on :7447 yet - start the simulation with rover_sim.sh first." >&2
    fi
    echo "rover_sim: shell ready (RMW=$RMW_IMPLEMENTATION, namespace=$ROVER_NAMESPACE, ws=$ROVER_WS)."
    return 0
fi

set -eo pipefail

BUILD=false
LAUNCH_ARGS=()
for arg in "$@"; do
    case "$arg" in
        --build) BUILD=true ;;
        -h | --help)
            sed -n '17,38p' "$(readlink -f "$0")" | sed 's/^# \{0,1\}//'
            exit 0
            ;;
        *) LAUNCH_ARGS+=("$arg") ;;
    esac
done

_rover_sim_setup_env

missing=()
for pkg in gz_ros2_control ros_gz_bridge ros_gz_sim; do
    ros2 pkg prefix "$pkg" > /dev/null 2>&1 || missing+=("ros-${ROS_DISTRO}-${pkg//_/-}")
done
if [ ${#missing[@]} -gt 0 ]; then
    echo "rover_sim: missing packages. Install them with:" >&2
    echo "  sudo apt install -y ${missing[*]}" >&2
    exit 1
fi

if [ "$BUILD" = true ]; then
    (
        cd "$ROVER_WS"
        colcon build --symlink-install --packages-up-to rover_metapackage rover_autonomy \
            --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
    )
    # shellcheck disable=SC1091
    source "$ROVER_WS/install/setup.bash"
fi

if ! ros2 pkg prefix rover_gazebo > /dev/null 2>&1; then
    echo "rover_sim: rover_gazebo is not built in $ROVER_WS - run: rover_sim.sh --build" >&2
    exit 1
fi

ROUTER_PID=""
cleanup() {
    if [ -n "$ROUTER_PID" ]; then
        kill "$ROUTER_PID" 2> /dev/null || true
        wait "$ROUTER_PID" 2> /dev/null || true
    fi
}
trap cleanup EXIT

if _rover_sim_router_running; then
    echo "rover_sim: reusing the zenoh router already listening on :7447."
else
    # Localhost only; multicast scouting is off in the default router config, so this router
    # cannot find or be found by the rover's.
    ZENOH_CONFIG_OVERRIDE='listen/endpoints=["tcp/127.0.0.1:7447"]' \
        ros2 run rmw_zenoh_cpp rmw_zenohd > "/tmp/rover_sim_zenohd_$$.log" 2>&1 &
    ROUTER_PID=$!
    for _ in $(seq 1 50); do
        _rover_sim_router_running && break
        if ! kill -0 "$ROUTER_PID" 2> /dev/null; then
            echo "rover_sim: zenoh router failed, see /tmp/rover_sim_zenohd_$$.log" >&2
            exit 1
        fi
        sleep 0.1
    done
    echo "rover_sim: zenoh router started (pid $ROUTER_PID, localhost:7447)."
fi

echo "rover_sim: namespace=$ROVER_NAMESPACE use_gps=${ROVER_USE_GPS:-false} ws=$ROVER_WS"
ros2 launch rover_gazebo simulation.launch.py "${LAUNCH_ARGS[@]}"
