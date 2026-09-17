cmake_minimum_required(VERSION 3.10)
project(rover_crsf_teleop)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wold-style-cast)
endif()

set(PACKAGE_DEPENDENCIES
    diagnostic_msgs
    diagnostic_updater
    lifecycle_msgs
    rclcpp
    rclcpp_lifecycle
    std_srvs
    std_msgs
    geometry_msgs
    rover_msgs
)

find_package(ament_cmake REQUIRED)
foreach(PACKAGE IN ITEMS ${PACKAGE_DEPENDENCIES})
  find_package(${PACKAGE} REQUIRED)
endforeach()

# Domain + application: the teleop rules AND the CRSF wire decoder, with no ROS and no OS
# dependencies. Linked by both the node and the unit tests, so the tests exercise exactly the
# code that runs on the rover. The decoder lives here because it is pure byte math: no clock, no
# syscalls, no ROS types. Freshness is LinkMonitor's job, not the parser's.
add_library(${PROJECT_NAME}_core STATIC
            src/domain/crsf/crc8.cpp
            src/domain/crsf/crsf_parser.cpp
            src/domain/stick_mapping.cpp
            src/domain/switch_debouncer.cpp
            src/domain/link_monitor.cpp
            src/domain/teleop_health.cpp
            src/application/teleop_use_case.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
                           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                           $<INSTALL_INTERFACE:include>
)
set_target_properties(${PROJECT_NAME}_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Infrastructure: the ROS adapters and the lifecycle node. No serial_driver / io_context / asio
# here - the UART belongs to serial_driver's serial_bridge node, which the launch file starts,
# so this package links none of its code.
add_library(${PROJECT_NAME}_ros STATIC
            src/infrastructure/rc_message_conversions.cpp
            src/infrastructure/ros2_velocity_command_publisher.cpp
            src/infrastructure/ros2_trigger_safety_switch.cpp
            src/infrastructure/rover_crsf_teleop_node.cpp
            src/infrastructure/teleop_diagnostics_conversions.cpp
)
target_link_libraries(${PROJECT_NAME}_ros PUBLIC ${PROJECT_NAME}_core)
target_link_libraries(${PROJECT_NAME}_ros PUBLIC
    ${diagnostic_msgs_TARGETS}
    ${diagnostic_updater_TARGETS}
    ${lifecycle_msgs_TARGETS}
    ${rclcpp_TARGETS}
    ${rclcpp_lifecycle_TARGETS}
    ${std_srvs_TARGETS}
    ${std_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${rover_msgs_TARGETS}
)

add_executable(${PROJECT_NAME}_node src/rover_crsf_teleop_node_main.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_ros)

install(DIRECTORY include/ DESTINATION include)

install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(${PACKAGE_DEPENDENCIES})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  # Unit: pure domain / application, no ROS (test/unit/).
  foreach(TEST_NAME IN ITEMS
          test_crc8
          test_crsf_parser
          test_stick_mapping
          test_switch_debouncer
          test_link_monitor
          test_teleop_health
          test_teleop_use_case)
    ament_add_gtest(${TEST_NAME} test/unit/${TEST_NAME}.cpp)
    target_link_libraries(${TEST_NAME} ${PROJECT_NAME}_core)
  endforeach()

  # Infrastructure: diagnostics formatting and message conversion, without a running node.
  foreach(TEST_NAME IN ITEMS
          test_teleop_diagnostics_conversions
          test_rc_message_conversions)
    ament_add_gtest(${TEST_NAME} test/unit/${TEST_NAME}.cpp)
    target_link_libraries(${TEST_NAME} ${PROJECT_NAME}_ros)
  endforeach()

  # Integration: the lifecycle node against real ROS topics and services (test/integration/).
  ament_add_gtest(test_teleop_node test/integration/test_teleop_node.cpp TIMEOUT 60)
  target_link_libraries(test_teleop_node ${PROJECT_NAME}_ros)
  target_link_libraries(test_teleop_node
      ${lifecycle_msgs_TARGETS}
  )

  # E2E: the node started from its launch description (test/e2e/).
  add_launch_test(test/e2e/test_teleop_launch.py TIMEOUT 60)
endif()

ament_package()
