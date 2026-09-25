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

# Which rover_utils headers may touch ROS, checked rather than trusted.
#
# include/rover_utils/ holds two halves under one include path, and the namespace says which:
#   - ROS-free, namespace rover_utils: only the C++ standard library, POSIX (unistd.h, sys/,
#     netinet/, arpa/), yaml-cpp and other ROS-free rover_utils headers, so a domain or
#     application layer can include them;
#   - ROS-coupled, namespace rover_utils::ros: the headers on ROS_HEADERS, infrastructure only.
# A header not on ROS_HEADERS is ROS-free by default: a new helper that needs rclcpp or a message
# type fails here until it is listed and put in rover_utils::ros.
#
# Reads #include and namespace lines, not the preprocessor: an include behind a macro is not seen.
# An allow-list, for the reason in rover_crsf_teleop/test/layer_purity.cmake.
#
# Run as: cmake -DSOURCE_DIR=<path to rover_utils> -P test/header_layers.cmake

cmake_minimum_required(VERSION 3.10)  # IN_LIST; quoted if() arguments are never variables

set(ROS_HEADERS
    parameter_utils.hpp
    ros_utils.hpp
    shutdown_gate.hpp)

set(ALLOWED_INCLUDES
    "^#[ \t]*include[ \t]*<[a-z_]+>"
    "^#[ \t]*include[ \t]*<(unistd|(sys|netinet|arpa)/[a-z_]+)\\.h>"
    "^#[ \t]*include[ \t]*[<\"]yaml-cpp/[a-z_/]+\\.h[>\"]")
set(OWN_INCLUDE "^#[ \t]*include[ \t]*\"rover_utils/([a-z0-9_/]+\\.hpp)\"")

set(INCLUDE_ROOT "${SOURCE_DIR}/include/rover_utils")
file(GLOB_RECURSE HEADERS RELATIVE "${INCLUDE_ROOT}" "${INCLUDE_ROOT}/*.hpp" "${INCLUDE_ROOT}/*.h")

if(HEADERS STREQUAL "")
  message(FATAL_ERROR "header layers: no headers under '${INCLUDE_ROOT}' - the check is looking "
                      "in the wrong place and would pass vacuously.")
endif()

foreach(LISTED IN LISTS ROS_HEADERS)
  if(NOT LISTED IN_LIST HEADERS)
    message(FATAL_ERROR "header layers: ROS_HEADERS lists '${LISTED}', which is not in "
                        "include/rover_utils/. Take it off the list.")
  endif()
endforeach()

set(VIOLATIONS "")
set(ROS_FREE_COUNT 0)

foreach(HEADER IN LISTS HEADERS)
  file(READ "${INCLUDE_ROOT}/${HEADER}" CONTENT)
  set(CONTENT "\n${CONTENT}")  # so the first line matches like every other

  set(NAMESPACES "")
  string(REGEX MATCHALL "\n[ \t]*namespace[ \t]+[A-Za-z0-9_:]+" NAMESPACE_LINES "${CONTENT}")
  foreach(NAMESPACE_LINE IN LISTS NAMESPACE_LINES)
    string(REGEX REPLACE "^[ \t\n]*namespace[ \t]+" "" NAMESPACE "${NAMESPACE_LINE}")
    list(APPEND NAMESPACES "${NAMESPACE}")
  endforeach()

  if(HEADER IN_LIST ROS_HEADERS)
    if(NOT "rover_utils::ros" IN_LIST NAMESPACES OR "rover_utils" IN_LIST NAMESPACES)
      list(APPEND VIOLATIONS "  ${HEADER}: on ROS_HEADERS, so namespace rover_utils::ros, not rover_utils")
    endif()
    continue()
  endif()

  math(EXPR ROS_FREE_COUNT "${ROS_FREE_COUNT} + 1")

  if(NOT "rover_utils" IN_LIST NAMESPACES)
    list(APPEND VIOLATIONS "  ${HEADER}: not on ROS_HEADERS, so namespace rover_utils")
  endif()
  foreach(NAMESPACE IN LISTS NAMESPACES)
    if(NAMESPACE MATCHES "^(rover_utils::)?ros$" OR NAMESPACE MATCHES "^(rover_utils::)?ros::")
      list(APPEND VIOLATIONS "  ${HEADER}: namespace ${NAMESPACE} in a header not on ROS_HEADERS")
    endif()
  endforeach()

  string(REGEX MATCHALL "\n[ \t]*#[ \t]*include[^\n]*" INCLUDE_LINES "${CONTENT}")
  foreach(INCLUDE_LINE IN LISTS INCLUDE_LINES)
    string(STRIP "${INCLUDE_LINE}" LINE)
    set(ALLOWED FALSE)
    foreach(PATTERN IN LISTS ALLOWED_INCLUDES)
      if(LINE MATCHES "${PATTERN}")
        set(ALLOWED TRUE)
      endif()
    endforeach()
    if(LINE MATCHES "${OWN_INCLUDE}")
      if(NOT CMAKE_MATCH_1 IN_LIST ROS_HEADERS)
        set(ALLOWED TRUE)
      endif()
    endif()
    if(NOT ALLOWED)
      list(APPEND VIOLATIONS "  ${HEADER}: ${LINE}")
    endif()
  endforeach()
endforeach()

if(NOT VIOLATIONS STREQUAL "")
  list(JOIN VIOLATIONS "\n" REPORT)
  message(FATAL_ERROR
          "A rover_utils header is on the wrong side of the ROS line.\n"
          "Headers not on ROS_HEADERS (test/header_layers.cmake) are ROS-free: namespace "
          "rover_utils, and only C++ standard, POSIX, yaml-cpp and ROS-free rover_utils includes. "
          "A header that needs ROS goes on ROS_HEADERS and into namespace rover_utils::ros.\n\n"
          "${REPORT}\n")
endif()

list(LENGTH ROS_HEADERS ROS_COUNT)
message(STATUS "header layers: ${ROS_FREE_COUNT} ROS-free and ${ROS_COUNT} ROS-coupled headers, "
               "each on its side.")
