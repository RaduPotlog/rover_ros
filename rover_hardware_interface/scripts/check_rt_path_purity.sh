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
# Fails if a known-blocking Modbus/Phidget-SDK symbol appears in code that runs on the RT control
# loop thread ticked by controller_manager (see .claude/rules/ros2_control_architecture.md §5):
#   - RoverSystem::read()/write() themselves (src/rover_system/rover_system.cpp).
#   - Every per-variant *_system.cpp file's RT-reachable virtual overrides - updateHwStates() and
#     getSpeedCmd() - since RoverSystem::read()/write() call these via virtual dispatch (see the
#     RT contract comment on the RoverSystem class in rover_system.hpp). This is scanned for every
#     file matching src/rover_system/*_system.cpp, not just rover_a1_system.cpp, so a future rover
#     variant (e.g. rover_b1_system.cpp) is covered automatically without editing this script.
#   - The RT-loop decision logic in src/application/rover_control_loop_use_case.cpp, since every
#     method on RoverControlLoopUseCase is reachable from the RT path.
#
# Deliberately NOT scanned: defineRoverDriver()/defineRoverController() and any *_system.cpp code
# reachable only from them, since those run during on_configure() (not on the RT path) and are
# expected to construct blocking backends (e.g. `RoverSafetyController(...)`, a real Modbus TCP
# connection). Also NOT scanned: the internals of rover_driver/, rover_safety_controller/,
# rover_modbus/, rover_sensors/ (e.g. RoverDriverInterface::getData(), RoverGpioPort::
# queryControlInterfaceIOStates() implementations) - a driver/adapter backend that put a blocking
# call inside one of ITS OWN RT-reachable methods would not be caught here. This is a substring
# scan of specific function bodies, NOT a real call-graph analysis: it catches a blocking call
# written directly in one of the scanned bodies, but not one added several call-levels deep, or
# hidden inside a new driver/adapter implementation. It complements, and does not replace, careful
# review of any change touching the RT path - see the file-level comment on this concern in
# rover_system.hpp. Invoked from CMakeLists.txt as a plain CTest add_test, mirroring
# check_domain_purity.sh.
set -euo pipefail

if [[ $# -ne 1 ]]; then
    echo "usage: $0 <package_source_dir>" >&2
    exit 2
fi

PKG_DIR="$1"
ROVER_SYSTEM_DIR="${PKG_DIR}/src/rover_system"
ROVER_SYSTEM_FILE="${ROVER_SYSTEM_DIR}/rover_system.cpp"
CONTROL_LOOP_USE_CASE_FILE="${PKG_DIR}/src/application/rover_control_loop_use_case.cpp"

if [[ ! -f "${ROVER_SYSTEM_FILE}" ]]; then
    echo "check_rt_path_purity: ${ROVER_SYSTEM_FILE} not found" >&2
    exit 1
fi

if [[ ! -f "${CONTROL_LOOP_USE_CASE_FILE}" ]]; then
    echo "check_rt_path_purity: ${CONTROL_LOOP_USE_CASE_FILE} not found" >&2
    exit 1
fi

# Every hardware_interface::SystemInterface implementation lives directly under
# src/rover_system/ as <variant>_system.cpp - rover_system.cpp itself (the generic base) plus one
# file per concrete rover variant (e.g. rover_a1_system.cpp).
mapfile -t SYSTEM_FILES < <(find "${ROVER_SYSTEM_DIR}" -maxdepth 1 -type f -name '*_system.cpp' | sort)

if [[ ${#SYSTEM_FILES[@]} -eq 0 ]]; then
    echo "check_rt_path_purity: no *_system.cpp files found under ${ROVER_SYSTEM_DIR}" >&2
    exit 1
fi

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
    # (and including) the first column-0 closing brace after it. Empty output if no match.
    local file="$1"
    local pattern="$2"
    awk -v pat="${pattern}" '
        $0 ~ pat { found = 1 }
        found { print }
        found && /^}/ { exit }
    ' "${file}"
}

status=0

check_body() {
    # Scans $1 (a function body, or "" if the function wasn't found in this file) for every
    # forbidden symbol, attributing violations to the human-readable label $2.
    local body="$1"
    local label="$2"

    if [[ -z "${body}" ]]; then
        return
    fi

    for symbol in "${FORBIDDEN_SYMBOLS[@]}"; do
        if grep -qF -- "${symbol}" <<< "${body}"; then
            echo "RT PATH PURITY VIOLATION: ${label} calls blocking symbol '${symbol}'" >&2
            status=1
        fi
    done
}

read_body=$(extract_body "${ROVER_SYSTEM_FILE}" 'return_type RoverSystem::read')
write_body=$(extract_body "${ROVER_SYSTEM_FILE}" 'return_type RoverSystem::write')

if [[ -z "${read_body}" || -z "${write_body}" ]]; then
    echo "check_rt_path_purity: could not locate read()/write() in ${ROVER_SYSTEM_FILE}" >&2
    exit 1
fi

check_body "${read_body}" "RoverSystem::read()"
check_body "${write_body}" "RoverSystem::write()"

# RT-reachable virtual overrides that a per-variant *_system.cpp defines - see the class-header
# comment above defineRoverDriver()/updateHwStates()/getSpeedCmd() declarations in
# rover_system.hpp for why each is RT-reachable via read()/write()'s virtual dispatch.
# No trailing "(" in these patterns - awk's dynamic regexps need it backslash-escaped, and the
# method name alone is already unambiguous (mirrors the parenthesis-free patterns used for
# RoverSystem::read/write above).
RT_REACHABLE_OVERRIDE_PATTERNS=(
    '::updateHwStates'
    '::getSpeedCmd'
)

for system_file in "${SYSTEM_FILES[@]}"; do
    if [[ "${system_file}" == "${ROVER_SYSTEM_FILE}" ]]; then
        continue  # read()/write() already scanned above.
    fi

    for pattern in "${RT_REACHABLE_OVERRIDE_PATTERNS[@]}"; do
        body=$(extract_body "${system_file}" "${pattern}")
        check_body "${body}" "${system_file} (${pattern})"
    done
done

check_body "$(cat "${CONTROL_LOOP_USE_CASE_FILE}")" "RoverControlLoopUseCase"

if [[ ${status} -eq 0 ]]; then
    echo "check_rt_path_purity: OK (${#SYSTEM_FILES[@]} *_system.cpp file(s) + control loop use case checked)"
fi

exit "${status}"
