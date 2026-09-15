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

# Powers off the ROS_CONTROLLER, the computer running the rover ROS 2 stack.
#
# rover_safety_node runs this script (with --no-e-stop, it trips the E-Stop itself) at the end of
# its RoverShutdown behavior tree. It also works on its own, e.g. when the ROS stack is down:
#
#   ros2 run rover_safety shutdown_ros_controller.sh --reason "Maintenance"

set -uo pipefail

readonly SCRIPT_NAME="shutdown_ros_controller"
readonly E_STOP_SERVICE="hardware_interface/sw_user_e_stop_set"
readonly E_STOP_TIMEOUT_S=5
readonly BALENA_HOST_DBUS_SOCKET="/host/run/dbus/system_bus_socket"
# Returned by a power-off method that is not available here, as opposed to one that failed.
readonly UNAVAILABLE=100

reason="${ROVER_SHUTDOWN_REASON:-Manual shutdown request}"
namespace="${ROVER_NAMESPACE:-}"
trip_e_stop=true
dry_run=false

usage() {
  cat <<USAGE
Usage: ${SCRIPT_NAME}.sh [options]

Trips the rover E-Stop, then powers off this computer with the first available method:
  1. balena Supervisor API   (BALENA_SUPERVISOR_ADDRESS and BALENA_SUPERVISOR_API_KEY set)
  2. systemd-logind (D-Bus)  (dbus-send; uses ${BALENA_HOST_DBUS_SOCKET} when present)
  3. systemctl poweroff

Options:
  --reason TEXT      Reason written to the log (default: \$ROVER_SHUTDOWN_REASON or "Manual shutdown request").
  --namespace NS     ROS namespace of the E-Stop service (default: \$ROVER_NAMESPACE).
  --no-e-stop        Do not call ${E_STOP_SERVICE}.
  --dry-run          Print what would be done without doing it.
  -h, --help         Show this help.

Exit status: 0 when the power-off request was accepted, 1 when every method failed, 2 on bad usage.
USAGE
}

log() { echo "[${SCRIPT_NAME}] $*"; }
warn() { echo "[${SCRIPT_NAME}] WARNING: $*" >&2; }
error() { echo "[${SCRIPT_NAME}] ERROR: $*" >&2; }

require_value() {
  if [ "$2" -lt 2 ]; then
    error "$1 requires a value"
    usage >&2
    exit 2
  fi
}

while [ $# -gt 0 ]; do
  case "$1" in
    --reason) require_value "$1" $#; reason="$2"; shift 2 ;;
    --reason=*) reason="${1#*=}"; shift ;;
    --namespace) require_value "$1" $#; namespace="$2"; shift 2 ;;
    --namespace=*) namespace="${1#*=}"; shift ;;
    --no-e-stop) trip_e_stop=false; shift ;;
    --dry-run) dry_run=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) error "Unknown option: $1"; usage >&2; exit 2 ;;
  esac
done

do_trip_e_stop() {
  local ns="${namespace#/}"
  ns="${ns%/}"
  local service="/${ns:+${ns}/}${E_STOP_SERVICE}"

  if ! command -v ros2 >/dev/null 2>&1; then
    warn "ros2 not found, cannot trip the E-Stop (${service}); continuing."
    return
  fi

  if $dry_run; then
    log "dry-run: timeout ${E_STOP_TIMEOUT_S} ros2 service call ${service} std_srvs/srv/Trigger {}"
    return
  fi

  log "Tripping the E-Stop (${service})."
  local output
  if output="$(timeout "${E_STOP_TIMEOUT_S}" ros2 service call "${service}" std_srvs/srv/Trigger '{}' 2>&1)" \
    && grep -q "success=True" <<<"${output}"; then
    log "E-Stop tripped."
  else
    warn "Could not trip the E-Stop; continuing with the power-off. Last output: ${output##*$'\n'}"
  fi
}

poweroff_balena_supervisor() {
  if [ -z "${BALENA_SUPERVISOR_ADDRESS:-}" ] || [ -z "${BALENA_SUPERVISOR_API_KEY:-}" ]; then
    return "${UNAVAILABLE}"
  fi
  if ! command -v curl >/dev/null 2>&1; then
    warn "balena Supervisor API configured but curl is missing."
    return "${UNAVAILABLE}"
  fi

  log "Method: balena Supervisor API (${BALENA_SUPERVISOR_ADDRESS}/v1/shutdown)."
  if $dry_run; then
    log "dry-run: curl -fsS -X POST ${BALENA_SUPERVISOR_ADDRESS}/v1/shutdown?apikey=<redacted>"
    return 0
  fi

  curl -fsS --max-time 5 -X POST -H "Content-Type: application/json" --data '{"force": false}' \
    "${BALENA_SUPERVISOR_ADDRESS}/v1/shutdown?apikey=${BALENA_SUPERVISOR_API_KEY}"
}

poweroff_logind_dbus() {
  if ! command -v dbus-send >/dev/null 2>&1; then
    return "${UNAVAILABLE}"
  fi
  if [ -z "${DBUS_SYSTEM_BUS_ADDRESS:-}" ] && [ -S "${BALENA_HOST_DBUS_SOCKET}" ]; then
    export DBUS_SYSTEM_BUS_ADDRESS="unix:path=${BALENA_HOST_DBUS_SOCKET}"
  fi
  if [ -z "${DBUS_SYSTEM_BUS_ADDRESS:-}" ] && [ ! -S /run/dbus/system_bus_socket ]; then
    return "${UNAVAILABLE}"
  fi

  local command=(dbus-send --system --print-reply --reply-timeout=5000 --dest=org.freedesktop.login1
    /org/freedesktop/login1 org.freedesktop.login1.Manager.PowerOff boolean:true)

  log "Method: systemd-logind over D-Bus (${DBUS_SYSTEM_BUS_ADDRESS:-unix:path=/run/dbus/system_bus_socket})."
  if $dry_run; then
    log "dry-run: ${command[*]}"
    return 0
  fi

  "${command[@]}"
}

poweroff_systemctl() {
  if ! command -v systemctl >/dev/null 2>&1; then
    return "${UNAVAILABLE}"
  fi

  log "Method: systemctl poweroff."
  if $dry_run; then
    log "dry-run: systemctl --no-ask-password poweroff"
    return 0
  fi

  systemctl --no-ask-password poweroff
}

log "Shutting down the ROS controller. Reason: ${reason}"

if $trip_e_stop; then
  do_trip_e_stop
fi

for method in poweroff_balena_supervisor poweroff_logind_dbus poweroff_systemctl; do
  "${method}"
  rc=$?
  if [ "${rc}" -eq 0 ]; then
    log "Power-off requested."
    exit 0
  fi
  if [ "${rc}" -ne "${UNAVAILABLE}" ]; then
    warn "${method} failed (exit code ${rc}); trying the next method."
  fi
done

error "No power-off method succeeded."
exit 1
