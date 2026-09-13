cmake_minimum_required(VERSION 3.8)
project(rover_crfs_teleop)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wold-style-cast)
endif()

set(PACKAGE_DEPENDENCIES
    rclcpp
    rclcpp_lifecycle
    std_srvs
    geometry_msgs
    crsf_receiver_msg
)

find_package(ament_cmake REQUIRED)
foreach(PACKAGE IN ITEMS ${PACKAGE_DEPENDENCIES})
  find_package(${PACKAGE} REQUIRED)
endforeach()

# Domain + application: the teleop rules, with no ROS dependencies. Linked by both the node and
# the unit tests, so the tests exercise exactly the code that runs on the rover.
add_library(${PROJECT_NAME}_core STATIC
            src/domain/stick_mapping.cpp
            src/domain/switch_debouncer.cpp
            src/domain/link_monitor.cpp
            src/application/teleop_use_case.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
                           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                           $<INSTALL_INTERFACE:include>
)
set_target_properties(${PROJECT_NAME}_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Infrastructure: the ROS adapters and the lifecycle node.
add_library(${PROJECT_NAME}_ros STATIC
            src/infrastructure/ros2_velocity_command_publisher.cpp
            src/infrastructure/ros2_trigger_safety_switch.cpp
            src/infrastructure/rover_crfs_teleop_node.cpp
)
target_link_libraries(${PROJECT_NAME}_ros PUBLIC ${PROJECT_NAME}_core)
target_link_libraries(${PROJECT_NAME}_ros PUBLIC
    ${rclcpp_TARGETS}
    ${rclcpp_lifecycle_TARGETS}
    ${std_srvs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${crsf_receiver_msg_TARGETS}
)

add_executable(${PROJECT_NAME}_node src/rover_crfs_teleop_node_main.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_ros)

install(DIRECTORY include/ DESTINATION include)

install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(${PACKAGE_DEPENDENCIES})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(lifecycle_msgs REQUIRED)

  # Unit: pure domain / application, no ROS (test/unit/).
  foreach(TEST_NAME IN ITEMS
          test_stick_mapping
          test_switch_debouncer
          test_link_monitor
          test_teleop_use_case)
    ament_add_gtest(${TEST_NAME} test/unit/${TEST_NAME}.cpp)
    target_link_libraries(${TEST_NAME} ${PROJECT_NAME}_core)
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
