cmake_minimum_required(VERSION 3.10)
project(rover_serial_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wold-style-cast)
endif()

set(PACKAGE_DEPENDENCIES
    rclcpp
    rclcpp_lifecycle
    lifecycle_msgs
    rclcpp_components
    std_msgs
    rover_io_context
)

find_package(ament_cmake REQUIRED)
foreach(PACKAGE IN ITEMS ${PACKAGE_DEPENDENCIES})
  find_package(${PACKAGE} REQUIRED)
endforeach()
find_package(rover_asio_cmake_module REQUIRED)
find_package(ASIO REQUIRED)

# Domain: the serial line configuration and the parsing of the five ROS parameters into
# it. No ROS, no ASIO, no OS - linked by the unit tests, which is the point of the split.
add_library(${PROJECT_NAME}_core STATIC
            src/domain/serial_port_config.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
                           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                           $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_core PUBLIC rover_io_context::rover_io_context_core)
set_target_properties(${PROJECT_NAME}_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Infrastructure: the ASIO UART adapter, the ASIO option translation, the message
# conversions and the lifecycle node.
add_library(${PROJECT_NAME}_ros STATIC
            src/infrastructure/asio_serial_options.cpp
            src/infrastructure/asio_serial_port.cpp
            src/infrastructure/serial_msg_conversions.cpp
            src/infrastructure/ros2_byte_publisher.cpp
            src/infrastructure/serial_bridge_node.cpp
)
target_link_libraries(${PROJECT_NAME}_ros PUBLIC
    ${PROJECT_NAME}_core
    rover_io_context::rover_io_context_asio
    ${rclcpp_TARGETS}
    ${rclcpp_lifecycle_TARGETS}
    ${lifecycle_msgs_TARGETS}
    ${std_msgs_TARGETS}
)
# SYSTEM so the house -Wold-style-cast / -Wpedantic do not fire inside ASIO's headers.
target_include_directories(${PROJECT_NAME}_ros SYSTEM PUBLIC
                           $<BUILD_INTERFACE:${ASIO_INCLUDE_DIRS}>
)
target_compile_definitions(${PROJECT_NAME}_ros PUBLIC ${ASIO_DEFINITIONS})
# Linked into the SHARED component library below.
set_target_properties(${PROJECT_NAME}_ros PROPERTIES POSITION_INDEPENDENT_CODE ON)

# The same node as an rclcpp_components plugin (rover::transport::serial::SerialBridgeNode), for
# launch files that load it into the container of the node it serves. The registration source
# must be compiled into this SHARED library itself - see the file. Plural macro: it registers
# without generating an executable, which would clash with the hand-written main below.
add_library(${PROJECT_NAME}_components SHARED src/infrastructure/serial_bridge_component.cpp)
target_link_libraries(${PROJECT_NAME}_components ${PROJECT_NAME}_ros ${rclcpp_components_TARGETS})
rclcpp_components_register_nodes(${PROJECT_NAME}_components
    "rover::transport::serial::SerialBridgeNode")

add_executable(rover_serial_bridge_node src/rover_serial_bridge_node_main.cpp)
target_link_libraries(rover_serial_bridge_node ${PROJECT_NAME}_ros)

install(DIRECTORY include/ DESTINATION include)

install(TARGETS ${PROJECT_NAME}_components
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

install(TARGETS rover_serial_bridge_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(${PACKAGE_DEPENDENCIES})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Pure domain - no ROS, no ASIO.
  ament_add_gtest(test_serial_port_config test/unit/test_serial_port_config.cpp)
  target_link_libraries(test_serial_port_config ${PROJECT_NAME}_core)

  # Infrastructure without a running node.
  foreach(TEST_NAME IN ITEMS
          test_asio_serial_options
          test_serial_msg_conversions)
    ament_add_gtest(${TEST_NAME} test/unit/${TEST_NAME}.cpp)
    target_link_libraries(${TEST_NAME} ${PROJECT_NAME}_ros)
  endforeach()

  # Integration: the ASIO adapter against the OS. Needs no real UART - see the file header.
  ament_add_gtest(test_asio_serial_port test/integration/test_asio_serial_port.cpp)
  target_link_libraries(test_asio_serial_port ${PROJECT_NAME}_ros)
endif()

ament_package()
