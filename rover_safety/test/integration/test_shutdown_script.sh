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

# Tests shutdown_ros_controller.sh against stub ros2/curl/dbus-send/systemctl binaries that only
# record how they were called. Usage: test_shutdown_script.sh <path to shutdown_ros_controller.sh>

set -uo pipefail

readonly SCRIPT="$1"
readonly WORK_DIR="$(mktemp -d)"
readonly STUB_DIR="${WORK_DIR}/bin"
readonly CALLS="${WORK_DIR}/calls.log"
trap 'rm -rf "${WORK_DIR}"' EXIT

failures=0
fail() { echo "FAIL: $*" >&2; failures=$((failures + 1)); }

mkdir -p "${STUB_DIR}"

# make_stub <name> <exit code> [stdout]
make_stub() {
  cat > "${STUB_DIR}/$1" <<STUB
#!/usr/bin/env bash
echo "$1 \$*" >> "${CALLS}"
printf '%s' "${3:-}"
exit $2
STUB
  chmod +x "${STUB_DIR}/$1"
}

reset_stubs() {
  rm -f "${STUB_DIR}"/* "${CALLS}"
  touch "${CALLS}"
  make_stub ros2 0 "std_srvs.srv.Trigger_Response(success=True, message='')"
  make_stub curl 0
  make_stub dbus-send 0
  make_stub systemctl 0
}

# run_script <args...>: runs the script with only the stubs and coreutils on PATH, no balena
# variables and a fake D-Bus address. Sets `output` and `rc`.
run_script() {
  output="$(env -i HOME="${WORK_DIR}" PATH="${STUB_DIR}:/usr/bin:/bin" \
    DBUS_SYSTEM_BUS_ADDRESS="unix:path=${WORK_DIR}/fake_bus" \
    ${EXTRA_ENV:-} bash "${SCRIPT}" "$@" 2>&1)"
  rc=$?
}

# The stubs must shadow the real binaries, or a test could power off the machine running it.
reset_stubs
for tool in ros2 curl dbus-send systemctl; do
  resolved="$(PATH="${STUB_DIR}:/usr/bin:/bin" command -v "${tool}")"
  if [ "${resolved}" != "${STUB_DIR}/${tool}" ]; then
    echo "FAIL: stub for ${tool} is not first on PATH (${resolved}); aborting." >&2
    exit 1
  fi
done

# --help and bad usage
reset_stubs
run_script --help
[ "${rc}" -eq 0 ] || fail "--help exit code ${rc}"
run_script --bogus
[ "${rc}" -eq 2 ] || fail "unknown option exit code ${rc}"
run_script --reason
[ "${rc}" -eq 2 ] || fail "--reason without value exit code ${rc}"
[ ! -s "${CALLS}" ] || fail "bad usage ran a tool: $(cat "${CALLS}")"

# Default: trips the namespaced E-Stop, then D-Bus power-off
reset_stubs
EXTRA_ENV="ROVER_NAMESPACE=/rover/" run_script --reason "unit test"
[ "${rc}" -eq 0 ] || fail "default run exit code ${rc}: ${output}"
grep -q "^ros2 service call /rover/hardware_interface/sw_user_e_stop_set std_srvs/srv/Trigger" "${CALLS}" \
  || fail "E-Stop not called on the namespaced service: $(cat "${CALLS}")"
grep -q "^dbus-send .*org.freedesktop.login1.Manager.PowerOff boolean:true" "${CALLS}" \
  || fail "D-Bus PowerOff not called: $(cat "${CALLS}")"
grep -q "^curl" "${CALLS}" && fail "curl called without balena variables"
grep -q "Reason: unit test" <<<"${output}" || fail "reason not logged: ${output}"

# --no-e-stop, reason from the environment
reset_stubs
EXTRA_ENV="ROVER_SHUTDOWN_REASON=from_env" run_script --no-e-stop
[ "${rc}" -eq 0 ] || fail "--no-e-stop exit code ${rc}"
grep -q "^ros2" "${CALLS}" && fail "--no-e-stop still called ros2"
grep -q "Reason: from_env" <<<"${output}" || fail "ROVER_SHUTDOWN_REASON not used: ${output}"

# A failing E-Stop does not block the power-off
reset_stubs
make_stub ros2 1 "service not available"
run_script
[ "${rc}" -eq 0 ] || fail "E-Stop failure blocked the power-off (exit ${rc})"
grep -q "^dbus-send" "${CALLS}" || fail "no power-off after E-Stop failure"

# balena Supervisor API is preferred when configured
reset_stubs
EXTRA_ENV="BALENA_SUPERVISOR_ADDRESS=http://127.0.0.1:48484 BALENA_SUPERVISOR_API_KEY=secretkey" run_script --no-e-stop
[ "${rc}" -eq 0 ] || fail "supervisor run exit code ${rc}"
grep -q "^curl .*http://127.0.0.1:48484/v1/shutdown?apikey=secretkey" "${CALLS}" || fail "supervisor not called: $(cat "${CALLS}")"
grep -q "^dbus-send" "${CALLS}" && fail "D-Bus called although the supervisor accepted"

# Supervisor failure falls back to D-Bus
reset_stubs
make_stub curl 22
EXTRA_ENV="BALENA_SUPERVISOR_ADDRESS=http://127.0.0.1:48484 BALENA_SUPERVISOR_API_KEY=secretkey" run_script --no-e-stop
[ "${rc}" -eq 0 ] || fail "fallback run exit code ${rc}"
grep -q "^dbus-send" "${CALLS}" || fail "no D-Bus fallback after supervisor failure"

# D-Bus failure falls back to systemctl
reset_stubs
make_stub dbus-send 1
run_script --no-e-stop
[ "${rc}" -eq 0 ] || fail "systemctl fallback exit code ${rc}"
grep -q "^systemctl --no-ask-password poweroff" "${CALLS}" || fail "systemctl not called: $(cat "${CALLS}")"

# Every method failing exits 1
reset_stubs
make_stub dbus-send 1
make_stub systemctl 1
run_script --no-e-stop
[ "${rc}" -eq 1 ] || fail "all-methods-failed exit code ${rc}"

# --dry-run runs nothing and redacts the API key
reset_stubs
EXTRA_ENV="BALENA_SUPERVISOR_ADDRESS=http://127.0.0.1:48484 BALENA_SUPERVISOR_API_KEY=secretkey" run_script --dry-run
[ "${rc}" -eq 0 ] || fail "dry-run exit code ${rc}"
[ ! -s "${CALLS}" ] || fail "dry-run ran a tool: $(cat "${CALLS}")"
grep -q "Method: balena Supervisor API" <<<"${output}" || fail "dry-run did not print the method: ${output}"
grep -q "secretkey" <<<"${output}" && fail "dry-run printed the API key"

if [ "${failures}" -ne 0 ]; then
  echo "${failures} check(s) failed." >&2
  exit 1
fi
echo "All shutdown script checks passed."
