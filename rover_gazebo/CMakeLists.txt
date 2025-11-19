cmake_minimum_required(VERSION 3.8)
project(rover_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wold-style-cast)
endif()

set(PACKAGE_DEPENDENCIES
    ament_cmake
    hardware_interface
    gz_ros2_control
    pluginlib
    rclcpp
    realtime_tools
    std_msgs
    std_srvs
)

foreach(PACKAGE IN ITEMS ${PACKAGE_DEPENDENCIES})
  find_package(${PACKAGE} REQUIRED)
endforeach()

install(DIRECTORY config launch 
        DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/setup_envs.sh.in")

ament_export_dependencies(${PACKAGE_DEPENDENCIES})

ament_package()