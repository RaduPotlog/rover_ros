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

ament_package()
