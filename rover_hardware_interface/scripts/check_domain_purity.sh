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
# Fails if any file under include/rover_hardware_interface/domain/ or src/domain/ includes a
# ROS, hardware_interface, or vendor-SDK header, or a non-domain header from this package. Domain
# code must stay pure, dependency-free C++ so it can be unit-tested without ROS running (see
# .claude/rules/clean_architecture.md) — this mechanically enforces that rule so a future change
# can't silently reintroduce a dependency. Invoked from CMakeLists.txt as a plain CTest add_test.
#
# Note on "infrastructure": this package has no single include/rover_hardware_interface/infrastructure/
# directory. Everything outside domain/ and application/ IS the infrastructure layer, split into
# subsystem-named directories instead (rover_system/, rover_driver/, rover_safety_controller/,
# rover_sensors/, rover_modbus/, system_ros_interface/) - one per real hardware/ROS boundary. This
# is a deliberate, reviewed choice (each directory maps 1:1 to a concrete adapter a reader needs to
# reason about independently), not a naming oversight, so don't go looking for an infrastructure/
# folder that doesn't exist, and don't add one purely to match the template's default shape - the
# boundary this script enforces is the include graph below, which is naming-scheme-agnostic.
set -euo pipefail

if [[ $# -ne 1 ]]; then
    echo "usage: $0 <package_source_dir>" >&2
    exit 2
fi

PKG_DIR="$1"
DOMAIN_DIRS=(
    "${PKG_DIR}/include/rover_hardware_interface/domain"
    "${PKG_DIR}/src/domain"
)

# In-project (quoted) includes from domain/ files may only reach other domain/ headers or the
# dependency-free utils.hpp - never rover_system/, rover_driver/, rover_safety_controller/,
# rover_sensors/, rover_modbus/, or system_ros_interface/, all of which are infrastructure.
ALLOWED_QUOTED_PATTERN='^rover_hardware_interface/(domain/[^/]+\.hpp$|utils\.hpp$)'

# Angle-bracket includes are forbidden if they name a ROS package, hardware_interface, or a
# vendor hardware SDK - domain code has no business knowing any of these exist.
FORBIDDEN_KEYWORDS=(
    rclcpp
    rclcpp_lifecycle
    rclcpp_components
    lifecycle_msgs
    hardware_interface
    pluginlib
    realtime_tools
    diagnostic_updater
    generate_parameter_library
    tf2
    geometry_msgs
    std_msgs
    std_srvs
    rover_msgs
    imu_filter_madgwick
    phidgets_api
    phidgets
    Phidget22
    modbus
    Modbus_Core
)

existing_dirs=()
for dir in "${DOMAIN_DIRS[@]}"; do
    if [[ -d "${dir}" ]]; then
        existing_dirs+=("${dir}")
    fi
done

if [[ ${#existing_dirs[@]} -eq 0 ]]; then
    echo "check_domain_purity: no domain directories found under ${DOMAIN_DIRS[*]}" >&2
    exit 1
fi

mapfile -t files < <(find "${existing_dirs[@]}" -type f \( -name '*.hpp' -o -name '*.cpp' \) | sort)

if [[ ${#files[@]} -eq 0 ]]; then
    echo "check_domain_purity: no domain files found under ${existing_dirs[*]}" >&2
    exit 1
fi

status=0

for file in "${files[@]}"; do
    while IFS= read -r include_line; do
        quoted=$(sed -n 's/^#include[[:space:]]*"\(.*\)"[[:space:]]*$/\1/p' <<< "${include_line}")
        angled=$(sed -n 's/^#include[[:space:]]*<\(.*\)>[[:space:]]*$/\1/p' <<< "${include_line}")

        if [[ -n "${quoted}" ]]; then
            if [[ ! "${quoted}" =~ ${ALLOWED_QUOTED_PATTERN} ]]; then
                echo "DOMAIN PURITY VIOLATION: ${file} includes non-domain project header '${quoted}'" >&2
                status=1
            fi
        elif [[ -n "${angled}" ]]; then
            for kw in "${FORBIDDEN_KEYWORDS[@]}"; do
                if [[ "${angled}" == *"${kw}"* ]]; then
                    echo "DOMAIN PURITY VIOLATION: ${file} includes forbidden header <${angled}>" >&2
                    status=1
                fi
            done
        fi
    done < <(grep -E '^[[:space:]]*#include' "${file}")
done

if [[ ${status} -eq 0 ]]; then
    echo "check_domain_purity: OK (${#files[@]} files checked)"
fi

exit "${status}"
