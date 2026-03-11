cmake_minimum_required(VERSION 3.8)
project(rover_battery)

set_target_properties(${TARGET} PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(PACKAGE_DEPENDENCIES
    ament_cmake
    ament_index_cpp
    rclcpp
    rclcpp_lifecycle
    realtime_tools
    diagnostic_updater
    std_msgs
    std_srvs
    sensor_msgs
    geometry_msgs
    rover_msgs
    CppLinuxSerial
)

foreach(PACKAGE IN ITEMS ${PACKAGE_DEPENDENCIES})
  find_package(${PACKAGE} REQUIRED)
endforeach()

include_directories(include)

add_executable(${PROJECT_NAME}_node
               src/rover_battery.cpp 
               src/rover_battery_node.cpp
               src/battery_publisher/battery_publisher.cpp
               src/battery_publisher/single_battery_publisher.cpp               
)

target_include_directories(${PROJECT_NAME}_node PUBLIC 
                           ${CMAKE_INSTALL_PREFIX}/include
)

target_link_libraries(${PROJECT_NAME}_node CppLinuxSerial)

ament_target_dependencies(${PROJECT_NAME}_node ${PACKAGE_DEPENDENCIES})

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(${PACKAGE_DEPENDENCIES})

ament_package()