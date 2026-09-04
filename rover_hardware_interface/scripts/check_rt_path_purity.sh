#!/usr/bin/env bash
#
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
#
# Fails if RoverSystem::read()/write() (src/rover_system/rover_system.cpp) or the RT-loop
# decision logic they delegate to (src/application/rover_control_loop_use_case.cpp - see
# RoverControlLoopUseCase) contain a call to a known-blocking Modbus/Phidget-SDK symbol.
# read()/write() run on the RT control loop thread ticked by controller_manager and must never
# block on network I/O or a synchronous SDK call (see .claude/rules/ros2_control_architecture.md
# §5). This is a substring scan (of read()/write()'s bodies specifically, and of the
# RoverControlLoopUseCase source file as a whole, since every method on it is reachable from the
# RT path), NOT a real call-graph analysis: it catches a blocking call written directly in one of
# the scanned bodies, but not one added several call-levels deep in a function they call. It
# complements, and does not replace, careful review of any change touching the RT path. Invoked
# from CMakeLists.txt as a plain CTest add_test, mirroring check_domain_purity.sh.
set -euo pipefail

if [[ $# -ne 1 ]]; then
    echo "usage: $0 <package_source_dir>" >&2
    exit 2
fi

PKG_DIR="$1"
ROVER_SYSTEM_FILE="${PKG_DIR}/src/rover_system/rover_system.cpp"
CONTROL_LOOP_USE_CASE_FILE="${PKG_DIR}/src/application/rover_control_loop_use_case.cpp"

for target_file in "${ROVER_SYSTEM_FILE}" "${CONTROL_LOOP_USE_CASE_FILE}"; do
    if [[ ! -f "${target_file}" ]]; then
        echo "check_rt_path_purity: ${target_file} not found" >&2
        exit 1
    fi
done

# Known-blocking symbols: raw Modbus TCP round-trips and synchronous Phidget SDK calls. Reads/
# writes to hardware on the RT path must instead go through the cached/try_lock ports
# (RoverGpioPort, RoverDriverInterface) and the Phidget SDK's _async entry points - see the RT
# contract comment on the RoverSystem class in rover_system.hpp.
FORBIDDEN_SYMBOLS=(
    awaitResponse
    sendRequest
    writeDiscreteCoil
    readDiscreteCoil
    readDiscreteContact
    "RoverSafetyController("
    "RoverModbus("
)

extract_body() {
    # Prints the lines from the first line matching the given function-signature pattern up to
    # (and including) the first column-0 closing brace after it.
    local pattern="$1"
    awk -v pat="${pattern}" '
        $0 ~ pat { found = 1 }
        found { print }
        found && /^}/ { exit }
    ' "${ROVER_SYSTEM_FILE}"
}

read_body=$(extract_body 'return_type RoverSystem::read')
write_body=$(extract_body 'return_type RoverSystem::write')

if [[ -z "${read_body}" || -z "${write_body}" ]]; then
    echo "check_rt_path_purity: could not locate read()/write() in ${ROVER_SYSTEM_FILE}" >&2
    exit 1
fi

control_loop_use_case_body=$(cat "${CONTROL_LOOP_USE_CASE_FILE}")

status=0

for symbol in "${FORBIDDEN_SYMBOLS[@]}"; do
    if grep -qF -- "${symbol}" <<< "${read_body}"; then
        echo "RT PATH PURITY VIOLATION: RoverSystem::read() calls blocking symbol '${symbol}'" >&2
        status=1
    fi
    if grep -qF -- "${symbol}" <<< "${write_body}"; then
        echo "RT PATH PURITY VIOLATION: RoverSystem::write() calls blocking symbol '${symbol}'" >&2
        status=1
    fi
    if grep -qF -- "${symbol}" <<< "${control_loop_use_case_body}"; then
        echo "RT PATH PURITY VIOLATION: RoverControlLoopUseCase calls blocking symbol '${symbol}'" >&2
        status=1
    fi
done

if [[ ${status} -eq 0 ]]; then
    echo "check_rt_path_purity: OK"
fi

exit "${status}"
