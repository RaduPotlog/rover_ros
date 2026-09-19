cmake_minimum_required(VERSION 3.10)
project(rover_diag_manager)

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
    rclcpp
    rover_msgs
    rover_utils
)

find_package(ament_cmake REQUIRED)
find_package(PkgConfig REQUIRED)
foreach(PACKAGE IN ITEMS ${PACKAGE_DEPENDENCIES})
  find_package(${PACKAGE} REQUIRED)
endforeach()

# cppuprofile is not a colcon/ament package: it must be built and `sudo make install`-ed
# system-wide first, per rover_ros/README.md's workspace-setup steps.
pkg_check_modules(CPPUPROFILE REQUIRED IMPORTED_TARGET cppuprofile)

generate_parameter_library(system_diag_params
                           src/system_diag_params.yaml
)

# Domain + application: plain C++, no ROS dependencies.
add_library(${PROJECT_NAME}_core STATIC
            src/domain/percentage.cpp
            src/domain/system_health_evaluator.cpp
            src/application/monitor_system_use_case.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
                           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# Infrastructure: ROS 2 adapters + cppuprofile/filesystem sensing.
add_library(${PROJECT_NAME}_ros STATIC
            src/infrastructure/linux_system_metrics_source.cpp
            src/infrastructure/ros2_system_status_publisher.cpp
            src/infrastructure/system_diag_node.cpp
            src/infrastructure/system_status_msg_conversions.cpp
)
target_link_libraries(${PROJECT_NAME}_ros
                      ${PROJECT_NAME}_core
                      system_diag_params
                      PkgConfig::CPPUPROFILE
)
target_include_directories(${PROJECT_NAME}_ros PUBLIC ${rover_utils_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_ros
    ${diagnostic_msgs_TARGETS}
    ${diagnostic_updater_TARGETS}
    ${rclcpp_TARGETS}
    ${rover_msgs_TARGETS}
)

add_executable(${PROJECT_NAME}_node src/rover_diag_manager.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_ros)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_percentage test/unit/test_percentage.cpp)
  target_link_libraries(test_percentage ${PROJECT_NAME}_core)

  ament_add_gtest(test_system_health_evaluator test/unit/test_system_health_evaluator.cpp)
  target_link_libraries(test_system_health_evaluator ${PROJECT_NAME}_core)

  ament_add_gtest(test_monitor_system_use_case test/unit/test_monitor_system_use_case.cpp)
  target_link_libraries(test_monitor_system_use_case ${PROJECT_NAME}_core)

  ament_add_gtest(test_system_status_msg_conversions test/unit/test_system_status_msg_conversions.cpp)
  target_link_libraries(test_system_status_msg_conversions ${PROJECT_NAME}_ros)

  ament_add_gtest(test_linux_system_metrics_source test/unit/test_linux_system_metrics_source.cpp)
  target_link_libraries(test_linux_system_metrics_source ${PROJECT_NAME}_ros)

  ament_add_gtest(test_system_diag_node test/integration/test_system_diag_node.cpp
                  TIMEOUT 60)
  target_link_libraries(test_system_diag_node ${PROJECT_NAME}_ros)
endif()

ament_export_dependencies(${PACKAGE_DEPENDENCIES})

ament_package()
