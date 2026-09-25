cmake_minimum_required(VERSION 3.10)
project(rover_battery)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED YES)
set(CMAKE_CXX_EXTENSIONS NO)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(PACKAGE_DEPENDENCIES
    rclcpp
    diagnostic_updater
    sensor_msgs
    udp_msgs
    rover_msgs
    rover_utils
)

find_package(ament_cmake REQUIRED)
foreach(PACKAGE IN ITEMS ${PACKAGE_DEPENDENCIES})
  find_package(${PACKAGE} REQUIRED)
endforeach()

# Domain + application: plain C++, no ROS dependencies.
add_library(${PROJECT_NAME}_core STATIC
            src/domain/battery_classifier.cpp
            src/application/monitor_battery_use_case.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
                           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# Infrastructure: ROS 2 adapters.
add_library(${PROJECT_NAME}_ros STATIC
            src/infrastructure/battery_msg_conversions.cpp
            src/infrastructure/ros2_battery_state_publisher.cpp
            src/infrastructure/rover_battery_node.cpp
)
target_link_libraries(${PROJECT_NAME}_ros ${PROJECT_NAME}_core)
target_include_directories(${PROJECT_NAME}_ros PUBLIC ${rover_utils_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_ros
    ${rclcpp_TARGETS}
    ${diagnostic_updater_TARGETS}
    ${sensor_msgs_TARGETS}
    ${udp_msgs_TARGETS}
    ${rover_msgs_TARGETS}
)

add_executable(${PROJECT_NAME}_node src/rover_battery.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_ros)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_battery_classifier test/unit/test_battery_classifier.cpp)
  target_link_libraries(test_battery_classifier ${PROJECT_NAME}_core)

  ament_add_gtest(test_monitor_battery_use_case test/unit/test_monitor_battery_use_case.cpp)
  target_link_libraries(test_monitor_battery_use_case ${PROJECT_NAME}_core)

  ament_add_gtest(test_battery_msg_conversions test/unit/test_battery_msg_conversions.cpp)
  target_link_libraries(test_battery_msg_conversions ${PROJECT_NAME}_ros)

  ament_add_gtest(test_rover_battery_node test/integration/test_rover_battery_node.cpp
                  TIMEOUT 60)
  target_link_libraries(test_rover_battery_node ${PROJECT_NAME}_ros)
endif()

ament_export_dependencies(${PACKAGE_DEPENDENCIES})

ament_package()
