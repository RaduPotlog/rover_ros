#!/usr/bin/env bash
#
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
#
# Fails if a file of this package's _core library (domain/ and application/) includes a header it
# must not. Mirrors rover_hardware_interface/scripts/check_domain_purity.sh. Invoked from
# CMakeLists.txt as a plain CTest add_test.
#
# The linker already keeps sockets out of _core: it links Modbus_Core, not Modbus_Tcp. This keeps
# the MB:: frame codec out too. ModbusTransportPort speaks DiscreteRequest/DiscreteReply
# (domain/discrete_transaction.hpp), and infrastructure/mb_frame_mapping is the only translation
# to and from MB::ModbusRequest/MB::ModbusResponse. The one MB:: type _core still uses is
# MB::ModbusException, DiscreteIoPort's documented failure type, which the client raises for empty
# or non-coil replies. So:
#   - domain/ includes no MB:: header at all;
#   - application/ headers include none either;
#   - application/ sources may include <MB/modbusException.hpp> and <MB/modbusUtils.hpp> (its
#     error and function codes), and nothing else from MB/.
set -euo pipefail

if [[ $# -ne 1 ]]; then
    echo "usage: $0 <package_source_dir>" >&2
    exit 2
fi

PKG_DIR="$1"
DOMAIN_DIRS=(
    "${PKG_DIR}/include/rover_modbus_driver/domain"
    "${PKG_DIR}/src/domain"
)
APPLICATION_DIRS=(
    "${PKG_DIR}/include/rover_modbus_driver/application"
    "${PKG_DIR}/src/application"
)
APPLICATION_SOURCE_DIR="${PKG_DIR}/src/application"

# Quoted includes: domain/ may reach only other domain/ headers; application/ may reach domain/ and
# application/ headers - never infrastructure/.
DOMAIN_QUOTED_PATTERN='^rover_modbus_driver/domain/[^/]+\.hpp$'
APPLICATION_QUOTED_PATTERN='^rover_modbus_driver/(domain|application)/[^/]+\.hpp$'

# Angle-bracket includes naming ROS or the vendored codec's package. "rover_modbus" also catches
# <rover_modbus_driver/...> spelled with angle brackets.
FORBIDDEN_KEYWORDS=(
    rclcpp
    rover_modbus
)

# The only MB:: headers an application/ source file may include.
ALLOWED_APPLICATION_SOURCE_MB_HEADERS=(
    MB/modbusException.hpp
    MB/modbusUtils.hpp
)

find_files() {
    local dirs=()
    local dir
    for dir in "$@"; do
        if [[ -d "${dir}" ]]; then
            dirs+=("${dir}")
        fi
    done

    if [[ ${#dirs[@]} -eq 0 ]]; then
        return
    fi

    find "${dirs[@]}" -type f \( -name '*.hpp' -o -name '*.cpp' \) | sort
}

mapfile -t domain_files < <(find_files "${DOMAIN_DIRS[@]}")
mapfile -t application_files < <(find_files "${APPLICATION_DIRS[@]}")

if [[ ${#domain_files[@]} -eq 0 ]]; then
    echo "check_core_purity: no domain files found under ${DOMAIN_DIRS[*]}" >&2
    exit 1
fi

if [[ ${#application_files[@]} -eq 0 ]]; then
    echo "check_core_purity: no application files found under ${APPLICATION_DIRS[*]}" >&2
    exit 1
fi

status=0

# check_file <layer> <file> <quoted_pattern> <mb_allowed: yes|no>
check_file() {
    local layer="$1"
    local file="$2"
    local quoted_pattern="$3"
    local mb_allowed="$4"
    local include_line quoted angled kw allowed header

    while IFS= read -r include_line; do
        quoted=$(sed -n 's/^[[:space:]]*#include[[:space:]]*"\(.*\)"[[:space:]]*$/\1/p' <<< "${include_line}")
        angled=$(sed -n 's/^[[:space:]]*#include[[:space:]]*<\(.*\)>[[:space:]]*$/\1/p' <<< "${include_line}")

        if [[ -n "${quoted}" ]]; then
            if [[ ! "${quoted}" =~ ${quoted_pattern} ]]; then
                echo "CORE PURITY VIOLATION: ${layer} file ${file} includes project header '${quoted}'" >&2
                status=1
            fi
        elif [[ -n "${angled}" ]]; then
            for kw in "${FORBIDDEN_KEYWORDS[@]}"; do
                if [[ "${angled}" == *"${kw}"* ]]; then
                    echo "CORE PURITY VIOLATION: ${layer} file ${file} includes forbidden header <${angled}>" >&2
                    status=1
                fi
            done

            if [[ "${angled}" == *"MB/"* ]]; then
                allowed=no
                if [[ "${mb_allowed}" == yes ]]; then
                    for header in "${ALLOWED_APPLICATION_SOURCE_MB_HEADERS[@]}"; do
                        if [[ "${angled}" == "${header}" ]]; then
                            allowed=yes
                        fi
                    done
                fi

                if [[ "${allowed}" != yes ]]; then
                    echo "CORE PURITY VIOLATION: ${layer} file ${file} includes MB:: codec header <${angled}>" >&2
                    status=1
                fi
            fi
        fi
    done < <(grep -E '^[[:space:]]*#include' "${file}" || true)
}

for file in "${domain_files[@]}"; do
    check_file domain "${file}" "${DOMAIN_QUOTED_PATTERN}" no
done

for file in "${application_files[@]}"; do
    mb_allowed=no
    if [[ "${file}" == "${APPLICATION_SOURCE_DIR}/"*.cpp ]]; then
        mb_allowed=yes
    fi

    check_file application "${file}" "${APPLICATION_QUOTED_PATTERN}" "${mb_allowed}"
done

if [[ ${status} -eq 0 ]]; then
    echo "check_core_purity: OK (${#domain_files[@]} domain, ${#application_files[@]} application files checked)"
fi

exit "${status}"
