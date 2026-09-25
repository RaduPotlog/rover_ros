cmake_minimum_required(VERSION 3.10)
project(rover_twist_mux)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED YES)
set(CMAKE_CXX_EXTENSIONS NO)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wold-style-cast)
endif()

set(PACKAGE_DEPENDENCIES
    diagnostic_msgs
    diagnostic_updater
    generate_parameter_library
    geometry_msgs
    rclcpp
    rover_msgs
    rover_utils
    std_msgs
)

find_package(ament_cmake REQUIRED)
foreach(PACKAGE IN ITEMS ${PACKAGE_DEPENDENCIES})
  find_package(${PACKAGE} REQUIRED)
endforeach()

generate_parameter_library(motion_lock_params
                           src/motion_lock_params.yaml
)
generate_parameter_library(command_freshness_params
                           src/command_freshness_params.yaml
)

# Domain: plain C++, no ROS dependencies.
add_library(${PROJECT_NAME}_core STATIC
            src/domain/motion_lock_policy.cpp
            src/domain/motion_lock_health.cpp
            src/domain/command_freshness_filter.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
                           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# Infrastructure: the ROS 2 adapters - gpio_state into the twist_mux lock topic, and the Driver
# UI's commands through the freshness filter into its twist_mux input.
add_library(${PROJECT_NAME}_ros STATIC
            src/infrastructure/motion_lock_node.cpp
            src/infrastructure/command_freshness_node.cpp
)
target_link_libraries(${PROJECT_NAME}_ros
                      ${PROJECT_NAME}_core
                      motion_lock_params
                      command_freshness_params
)
target_include_directories(${PROJECT_NAME}_ros PUBLIC ${rover_utils_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_ros
    ${diagnostic_msgs_TARGETS}
    ${diagnostic_updater_TARGETS}
    ${geometry_msgs_TARGETS}
    ${rclcpp_TARGETS}
    ${rover_msgs_TARGETS}
    ${std_msgs_TARGETS}
)

add_executable(motion_lock_node src/motion_lock.cpp)
target_link_libraries(motion_lock_node ${PROJECT_NAME}_ros)

add_executable(command_freshness_node src/command_freshness.cpp)
target_link_libraries(command_freshness_node ${PROJECT_NAME}_ros)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

install(TARGETS motion_lock_node command_freshness_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_motion_lock_policy test/unit/test_motion_lock_policy.cpp)
  target_link_libraries(test_motion_lock_policy ${PROJECT_NAME}_core)

  ament_add_gtest(test_motion_lock_health test/unit/test_motion_lock_health.cpp)
  target_link_libraries(test_motion_lock_health ${PROJECT_NAME}_core)

  ament_add_gtest(test_command_freshness_filter test/unit/test_command_freshness_filter.cpp)
  target_link_libraries(test_command_freshness_filter ${PROJECT_NAME}_core)

  # ROS-level: that the fail-safe behaviour the two suites above describe actually reaches the
  # motion_lock topic twist_mux consumes. The QoS, topic names and both-topics-required rule all
  # live in the node, and a mistake in any of them yields a lock that never asserts.
  ament_add_gtest(test_motion_lock_node test/integration/test_motion_lock_node.cpp)
  target_link_libraries(test_motion_lock_node ${PROJECT_NAME}_ros ${PROJECT_NAME}_core)

  # ROS-level: that fresh commands really reach the twist_mux input and late ones do not.
  ament_add_gtest(test_command_freshness_node test/integration/test_command_freshness_node.cpp)
  target_link_libraries(test_command_freshness_node ${PROJECT_NAME}_ros ${PROJECT_NAME}_core)

  # Config: the arbitration order the README promises (RC > joystick > Driver UI > Nav 2, and the
  # E-Stop lock above all of them) exists only as the numbers twist_mux reads from
  # config/rover_twist_mux.yaml. This pins their relative order.
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_twist_mux_priorities test/unit/test_twist_mux_priorities.py)
endif()

ament_export_dependencies(${PACKAGE_DEPENDENCIES})

ament_package()
