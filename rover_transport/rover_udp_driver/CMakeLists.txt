cmake_minimum_required(VERSION 3.10)
project(rover_udp_driver)

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
    udp_msgs
    rover_io_context
)

find_package(ament_cmake REQUIRED)
foreach(PACKAGE IN ITEMS ${PACKAGE_DEPENDENCIES})
  find_package(${PACKAGE} REQUIRED)
endforeach()
find_package(rover_asio_cmake_module REQUIRED)
find_package(ASIO REQUIRED)

# Domain: the validated endpoint. No ROS, no ASIO, no OS.
add_library(${PROJECT_NAME}_core STATIC
            src/domain/udp_endpoint.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
                           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                           $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_core PUBLIC rover_io_context::rover_io_context_core)
set_target_properties(${PROJECT_NAME}_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Infrastructure: the ASIO socket adapter, the packet conversions and the two lifecycle
# nodes.
add_library(${PROJECT_NAME}_ros STATIC
            src/infrastructure/asio_udp_socket.cpp
            src/infrastructure/udp_packet_conversions.cpp
            src/infrastructure/ros2_udp_packet_publisher.cpp
            src/infrastructure/udp_receiver_node.cpp
            src/infrastructure/udp_sender_node.cpp
)
target_link_libraries(${PROJECT_NAME}_ros PUBLIC
    ${PROJECT_NAME}_core
    rover_io_context::rover_io_context_asio
    ${rclcpp_TARGETS}
    ${rclcpp_lifecycle_TARGETS}
    ${lifecycle_msgs_TARGETS}
    ${udp_msgs_TARGETS}
)
# SYSTEM so the house -Wold-style-cast / -Wpedantic do not fire inside ASIO's headers.
target_include_directories(${PROJECT_NAME}_ros SYSTEM PUBLIC
                           $<BUILD_INTERFACE:${ASIO_INCLUDE_DIRS}>
)
target_compile_definitions(${PROJECT_NAME}_ros PUBLIC ${ASIO_DEFINITIONS})
# Linked into the SHARED component library below.
set_target_properties(${PROJECT_NAME}_ros PROPERTIES POSITION_INDEPENDENT_CODE ON)

# The same two nodes as rclcpp_components plugins (rover::transport::udp::UdpReceiverNode,
# ::UdpSenderNode), for launch files that load them into the container of the node they serve.
# The registration source must be compiled into this SHARED library itself - see the file.
# Plural macro: it registers without generating executables, which would clash with the
# hand-written mains below.
add_library(${PROJECT_NAME}_components SHARED src/infrastructure/udp_components.cpp)
target_link_libraries(${PROJECT_NAME}_components ${PROJECT_NAME}_ros ${rclcpp_components_TARGETS})
rclcpp_components_register_nodes(${PROJECT_NAME}_components
    "rover::transport::udp::UdpReceiverNode"
    "rover::transport::udp::UdpSenderNode")

foreach(EXE IN ITEMS
        rover_udp_receiver_node
        rover_udp_sender_node
        rover_udp_bridge_node)
  add_executable(${EXE} src/${EXE}_main.cpp)
  target_link_libraries(${EXE} ${PROJECT_NAME}_ros)
endforeach()

install(DIRECTORY include/ DESTINATION include)

install(TARGETS ${PROJECT_NAME}_components
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

install(TARGETS
        rover_udp_receiver_node
        rover_udp_sender_node
        rover_udp_bridge_node
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(${PACKAGE_DEPENDENCIES})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Pure domain - no ROS, no ASIO.
  ament_add_gtest(test_udp_endpoint test/unit/test_udp_endpoint.cpp)
  target_link_libraries(test_udp_endpoint ${PROJECT_NAME}_core)

  # Infrastructure without a socket.
  ament_add_gtest(test_udp_packet_conversions test/unit/test_udp_packet_conversions.cpp)
  target_link_libraries(test_udp_packet_conversions ${PROJECT_NAME}_ros)

  # Integration: real loopback sockets on per-process ports.
  ament_add_gtest(test_asio_udp_socket test/integration/test_asio_udp_socket.cpp TIMEOUT 60)
  target_link_libraries(test_asio_udp_socket ${PROJECT_NAME}_ros)

  # Integration: the two lifecycle nodes against localhost sockets.
  ament_add_gtest(test_udp_nodes_lifecycle test/integration/test_udp_nodes_lifecycle.cpp
                  TIMEOUT 60)
  target_link_libraries(test_udp_nodes_lifecycle ${PROJECT_NAME}_ros)
endif()

ament_package()
