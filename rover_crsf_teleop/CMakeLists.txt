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
    yaml-cpp
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
            src/domain/rc_calibration.cpp
            src/domain/safety_io_flags.cpp
            src/domain/stick_mapping.cpp
            src/domain/rim_speed_limit.cpp
            src/domain/switch_debouncer.cpp
            src/domain/link_monitor.cpp
            src/domain/teleop_health.cpp
            src/application/calibration_use_case.cpp
            src/application/teleop_use_case.cpp
            src/application/teleop_config_validation.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
                           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                           $<INSTALL_INTERFACE:include>
)
set_target_properties(${PROJECT_NAME}_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Infrastructure, part one: the adapters that implement the domain ports plus the two pure
# translation units (domain <-> rover_msgs, health level -> diagnostic level). No node here, so
# the adapter tests below can link this without dragging the whole lifecycle node in with them.
#
# No rover_serial_driver / rover_io_context / asio anywhere in this package - the UART belongs to
# rover_serial_driver's rover_serial_bridge_node, which the launch file starts, so this package
# links none of its code.
add_library(${PROJECT_NAME}_adapters STATIC
            src/infrastructure/rc_message_conversions.cpp
            src/infrastructure/ros2_velocity_command_publisher.cpp
            src/infrastructure/ros2_trigger_safety_switch.cpp
            src/infrastructure/teleop_diagnostics_conversions.cpp
            src/infrastructure/yaml_calibration_store.cpp
)
target_link_libraries(${PROJECT_NAME}_adapters PUBLIC ${PROJECT_NAME}_core)
target_link_libraries(${PROJECT_NAME}_adapters PUBLIC
    ${diagnostic_msgs_TARGETS}
    ${diagnostic_updater_TARGETS}
    ${lifecycle_msgs_TARGETS}
    ${rclcpp_TARGETS}
    ${rclcpp_lifecycle_TARGETS}
    ${std_srvs_TARGETS}
    ${std_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${rover_msgs_TARGETS}
    yaml-cpp
)
set_target_properties(${PROJECT_NAME}_adapters PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Infrastructure, part two: the lifecycle node that composes all of the above.
add_library(${PROJECT_NAME}_ros STATIC
            src/infrastructure/rover_crsf_teleop_node.cpp
)
target_link_libraries(${PROJECT_NAME}_ros PUBLIC ${PROJECT_NAME}_adapters)

add_executable(${PROJECT_NAME}_node src/rover_crsf_teleop_node_main.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_ros)

# No install(DIRECTORY include/ ...) on purpose: _core / _adapters / _ros are STATIC and are
# neither installed nor exported, so shipping the headers would advertise an interface that
# compiles downstream and then fails to link. Nothing outside this package includes them. If a
# consumer ever appears, install and export the libraries first, then put the headers back.
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
          test_rim_speed_limit
          test_switch_debouncer
          test_rc_calibration
          test_safety_io_flags
          test_calibration_use_case
          test_link_monitor
          test_teleop_health
          test_teleop_use_case
          test_teleop_config_validation)
    ament_add_gtest(${TEST_NAME} test/unit/${TEST_NAME}.cpp)
    target_link_libraries(${TEST_NAME} ${PROJECT_NAME}_core)
  endforeach()

  # Infrastructure: diagnostics formatting, message conversion and the calibration file store.
  # Linked against _adapters, not _ros: none of them needs the node, and linking it would mean a
  # change to the node could break a test that has nothing to do with it.
  foreach(TEST_NAME IN ITEMS
          test_teleop_diagnostics_conversions
          test_rc_message_conversions
          test_yaml_calibration_store)
    ament_add_gtest(${TEST_NAME} test/unit/${TEST_NAME}.cpp)
    target_link_libraries(${TEST_NAME} ${PROJECT_NAME}_adapters)
  endforeach()

  # Integration: the lifecycle node against real ROS topics and services (test/integration/).
  ament_add_gtest(test_teleop_node test/integration/test_teleop_node.cpp TIMEOUT 60)
  target_link_libraries(test_teleop_node ${PROJECT_NAME}_ros)
  target_link_libraries(test_teleop_node
      ${lifecycle_msgs_TARGETS}
  )

  # The layer boundary, checked rather than trusted: domain/ and application/ must not include
  # ROS, yaml-cpp or the filesystem. See test/layer_purity.cmake for why linking alone is not
  # enough to catch it.
  add_test(NAME test_layer_purity
           COMMAND ${CMAKE_COMMAND} -DSOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}
                   -P ${CMAKE_CURRENT_SOURCE_DIR}/test/layer_purity.cmake)

  # E2E: the node started from its launch description (test/e2e/).
  add_launch_test(test/e2e/test_teleop_launch.py TIMEOUT 60)
endif()

ament_package()
