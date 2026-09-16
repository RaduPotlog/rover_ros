cmake_minimum_required(VERSION 3.10)
project(rover_lidar)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED YES)
set(CMAKE_CXX_EXTENSIONS NO)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(PACKAGE_DEPENDENCIES
    rclcpp
    diagnostic_msgs
    diagnostic_updater
    rcl_interfaces
    sensor_msgs
)

find_package(ament_cmake REQUIRED)
foreach(PACKAGE IN ITEMS ${PACKAGE_DEPENDENCIES})
  find_package(${PACKAGE} REQUIRED)
endforeach()

# Domain + application: plain C++, no ROS dependencies.
add_library(${PROJECT_NAME}_core STATIC
            src/domain/lidar_health_evaluator.cpp
            src/application/monitor_lidar_use_case.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
                           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# Infrastructure: ROS 2 adapters.
add_library(${PROJECT_NAME}_ros STATIC
            src/infrastructure/ros2_lidar_health_publisher.cpp
            src/infrastructure/rover_lidar_node.cpp
)
target_link_libraries(${PROJECT_NAME}_ros ${PROJECT_NAME}_core)
target_link_libraries(${PROJECT_NAME}_ros
    ${rclcpp_TARGETS}
    ${diagnostic_msgs_TARGETS}
    ${diagnostic_updater_TARGETS}
    ${rcl_interfaces_TARGETS}
    ${sensor_msgs_TARGETS}
)

add_executable(${PROJECT_NAME}_node src/rover_lidar.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_ros)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_lidar_health_evaluator test/unit/test_lidar_health_evaluator.cpp)
  target_link_libraries(test_lidar_health_evaluator ${PROJECT_NAME}_core)
endif()

ament_export_dependencies(${PACKAGE_DEPENDENCIES})

ament_package()
