cmake_minimum_required(VERSION 3.13)
project(rover_modbus)

# Modified 2026 by Mechatronics Academy: converted from a plain-CMake vendored project
# into an ament_cmake package. Previously this had to be built and `sudo make install`ed
# to /usr/local by hand before colcon could build anything that depended on it (see
# rover_ros/README.md and the platform Dockerfile). As an ament package it is ordered by
# colcon from the consumer's <depend> alone, so that manual step is gone.
#
# Also removed here: the MODBUS_EXAMPLE / MODBUS_TESTS / MODBUS_SERIAL_COMMUNICATION
# options. All three pointed at directories deleted by fork commit c5d53cb, so
# -DMODBUS_TESTS=ON failed configure outright.

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(MSVC)
    add_compile_options(/W3 /MT)
    # Otherwise, MSVC complains about strncopy
    add_definitions(-D_CRT_SECURE_NO_WARNINGS)
else()
    add_compile_options(-Wall -Wextra -Werror -pedantic)
endif()

find_package(ament_cmake REQUIRED)

add_subdirectory(src)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

# Modified 2026 by Mechatronics Academy: added. Fork commit c5d53cb deleted the upstream
# test suite, so the fixes listed under "Local changes to this fork" in README.md had no
# test in the package that owns them. Built with the -Werror flags above.
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # The frame codec alone: CRC, request encoding, response decoding. Modbus_Core only.
  ament_add_gtest(test_modbus_codec test/unit/test_modbus_codec.cpp)
  target_link_libraries(test_modbus_codec Modbus_Core)

  # The POSIX client and server: framing and error paths over socketpair(), connect/bind/
  # accept failures over loopback on kernel-assigned ports.
  ament_add_gtest(test_modbus_tcp test/integration/test_modbus_tcp.cpp TIMEOUT 60)
  target_link_libraries(test_modbus_tcp Modbus_Tcp)
endif()

ament_package()
