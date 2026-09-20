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

# Fails if anything in domain/ or application/ includes anything but the C++ standard library
# and this package's own headers - so no ROS, no yaml-cpp, no filesystem.
#
# rover_crsf_teleop_core links none of those, but that only catches a symbol that is actually
# called - a header-only include (an rclcpp::Duration overload on a config struct, say) would
# compile quietly and the layering would be gone with nothing to show for it. This is the check
# that says so out loud. Run as: cmake -DSOURCE_DIR=<pkg> -P test/layer_purity.cmake

# An allow-list, not a deny-list. The previous version named the things to reject, and the names
# never covered the space: "rclcpp/" missed rclcpp_action/ and rclcpp_components/, and
# "[a-z_]*_msgs/" missed builtin_interfaces/, tf2_ros/, std_srvs/ and every ROS package that is
# not spelled *_msgs. Nothing violated it, but the next dependency to arrive would have walked
# straight through. Saying what these layers *may* include closes the whole class instead.
#
# Permitted: a bare C++ standard header, or this package's own headers. Everything else - any ROS
# package, yaml-cpp, a C system header, a third-party library - is a violation by default.
set(ALLOWED_SYSTEM_INCLUDE "^[ \t]*#[ \t]*include[ \t]*<[a-z_]+>[ \t]*$")
set(ALLOWED_OWN_INCLUDE "^[ \t]*#[ \t]*include[ \t]*\"rover_crsf_teleop/[a-z0-9_/]+\\.hpp\"[ \t]*$")

# Standard headers that are still out: these layers do no I/O and touch no filesystem. That work
# belongs to an adapter in infrastructure/ behind a port in domain/ports.hpp.
set(FORBIDDEN_SYSTEM_INCLUDES
    "^[ \t]*#[ \t]*include[ \t]*<fstream>"
    "^[ \t]*#[ \t]*include[ \t]*<filesystem>"
    "^[ \t]*#[ \t]*include[ \t]*<iostream>"
    "^[ \t]*#[ \t]*include[ \t]*<cstdio>"
)

set(ANY_INCLUDE "^[ \t]*#[ \t]*include")

file(GLOB_RECURSE PURE_SOURCES
     "${SOURCE_DIR}/include/rover_crsf_teleop/domain/*"
     "${SOURCE_DIR}/include/rover_crsf_teleop/application/*"
     "${SOURCE_DIR}/src/domain/*"
     "${SOURCE_DIR}/src/application/*"
)

if(PURE_SOURCES STREQUAL "")
  message(FATAL_ERROR "layer purity: found no domain/application sources under '${SOURCE_DIR}' - "
                      "the check is looking in the wrong place and would pass vacuously.")
endif()

set(VIOLATIONS "")

foreach(SOURCE IN LISTS PURE_SOURCES)
  file(STRINGS "${SOURCE}" LINES)
  foreach(LINE IN LISTS LINES)
    if(NOT LINE MATCHES "${ANY_INCLUDE}")
      continue()
    endif()

    set(DENIED FALSE)
    foreach(PATTERN IN LISTS FORBIDDEN_SYSTEM_INCLUDES)
      if(LINE MATCHES "${PATTERN}")
        set(DENIED TRUE)
      endif()
    endforeach()

    if(DENIED)
      list(APPEND VIOLATIONS "  ${SOURCE}: ${LINE}")
    elseif(NOT LINE MATCHES "${ALLOWED_SYSTEM_INCLUDE}"
           AND NOT LINE MATCHES "${ALLOWED_OWN_INCLUDE}")
      list(APPEND VIOLATIONS "  ${SOURCE}: ${LINE}")
    endif()
  endforeach()
endforeach()

list(LENGTH PURE_SOURCES CHECKED)

if(NOT VIOLATIONS STREQUAL "")
  list(JOIN VIOLATIONS "\n" REPORT)
  message(FATAL_ERROR
          "domain/ and application/ may include only C++ standard headers (no <fstream>, "
          "<filesystem>, <iostream> or <cstdio>) and \"rover_crsf_teleop/...\" headers.\n"
          "Put the adapter in infrastructure/ and a port in domain/ports.hpp instead.\n\n"
          "${REPORT}\n")
endif()

message(STATUS "layer purity: ${CHECKED} domain/application files, only std and own includes.")
