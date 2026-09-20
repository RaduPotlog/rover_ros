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

# Fails if anything in domain/ or application/ includes ROS, yaml-cpp or the filesystem.
#
# rover_crsf_teleop_core links none of those, but that only catches a symbol that is actually
# called - a header-only include (an rclcpp::Duration overload on a config struct, say) would
# compile quietly and the layering would be gone with nothing to show for it. This is the check
# that says so out loud. Run as: cmake -DSOURCE_DIR=<pkg> -P test/layer_purity.cmake

set(FORBIDDEN_INCLUDES
    "#[ \t]*include[ \t]*[<\"]rclcpp/"
    "#[ \t]*include[ \t]*[<\"]rclcpp_lifecycle/"
    "#[ \t]*include[ \t]*[<\"]lifecycle_msgs/"
    "#[ \t]*include[ \t]*[<\"][a-z_]*_msgs/"
    "#[ \t]*include[ \t]*[<\"]yaml-cpp/"
    "#[ \t]*include[ \t]*[<\"]diagnostic_updater/"
    "#[ \t]*include[ \t]*<fstream>"
    "#[ \t]*include[ \t]*<filesystem>"
)

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
    foreach(PATTERN IN LISTS FORBIDDEN_INCLUDES)
      if(LINE MATCHES "${PATTERN}")
        list(APPEND VIOLATIONS "  ${SOURCE}: ${LINE}")
      endif()
    endforeach()
  endforeach()
endforeach()

list(LENGTH PURE_SOURCES CHECKED)

if(NOT VIOLATIONS STREQUAL "")
  list(JOIN VIOLATIONS "\n" REPORT)
  message(FATAL_ERROR
          "domain/ and application/ must not depend on ROS, yaml-cpp or the filesystem.\n"
          "Put the adapter in infrastructure/ and a port in domain/ports.hpp instead.\n\n"
          "${REPORT}\n")
endif()

message(STATUS "layer purity: ${CHECKED} domain/application files, no ROS or OS includes.")
