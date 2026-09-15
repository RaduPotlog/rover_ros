cmake_minimum_required(VERSION 3.10)
project(rover_gps)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED YES)
set(CMAKE_CXX_EXTENSIONS NO)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(PACKAGE_DEPENDENCIES
    rclcpp
    builtin_interfaces
    diagnostic_msgs
    diagnostic_updater
    nav_msgs
    rcl_interfaces
    sensor_msgs
    std_srvs
)

find_package(ament_cmake REQUIRED)
foreach(PACKAGE IN ITEMS ${PACKAGE_DEPENDENCIES})
  find_package(${PACKAGE} REQUIRED)
endforeach()

# Domain + application: plain C++, no ROS dependencies.
add_library(${PROJECT_NAME}_core STATIC
            src/domain/geo_math.cpp
            src/domain/gps_health_evaluator.cpp
            src/domain/heading_alignment_estimator.cpp
            src/application/monitor_gps_use_case.cpp
            src/application/align_heading_use_case.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
                           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# Infrastructure: ROS 2 adapters.
add_library(${PROJECT_NAME}_ros STATIC
            src/infrastructure/gps_msg_conversions.cpp
            src/infrastructure/ros2_gps_health_publisher.cpp
            src/infrastructure/ros2_heading_publisher.cpp
            src/infrastructure/rover_gps_node.cpp
)
target_link_libraries(${PROJECT_NAME}_ros ${PROJECT_NAME}_core)
target_link_libraries(${PROJECT_NAME}_ros
    ${rclcpp_TARGETS}
    ${builtin_interfaces_TARGETS}
    ${diagnostic_msgs_TARGETS}
    ${diagnostic_updater_TARGETS}
    ${nav_msgs_TARGETS}
    ${rcl_interfaces_TARGETS}
    ${sensor_msgs_TARGETS}
    ${std_srvs_TARGETS}
)

add_executable(${PROJECT_NAME}_node src/rover_gps.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_ros)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_geo_math test/unit/test_geo_math.cpp)
  target_link_libraries(test_geo_math ${PROJECT_NAME}_core)

  ament_add_gtest(test_gps_health_evaluator test/unit/test_gps_health_evaluator.cpp)
  target_link_libraries(test_gps_health_evaluator ${PROJECT_NAME}_core)

  ament_add_gtest(test_heading_alignment_estimator test/unit/test_heading_alignment_estimator.cpp)
  target_link_libraries(test_heading_alignment_estimator ${PROJECT_NAME}_core)

  ament_add_gtest(test_gps_use_cases test/unit/test_gps_use_cases.cpp)
  target_link_libraries(test_gps_use_cases ${PROJECT_NAME}_core)

  ament_add_gtest(test_gps_msg_conversions test/unit/test_gps_msg_conversions.cpp)
  target_link_libraries(test_gps_msg_conversions ${PROJECT_NAME}_ros)

  ament_add_gtest(test_rover_gps_node test/integration/test_rover_gps_node.cpp
                  TIMEOUT 60)
  target_link_libraries(test_rover_gps_node ${PROJECT_NAME}_ros)
endif()

ament_export_dependencies(${PACKAGE_DEPENDENCIES})

ament_package()
