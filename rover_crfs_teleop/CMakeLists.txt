cmake_minimum_required(VERSION 3.8)
project(rover_crfs_teleop)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wold-style-cast)
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
    geometry_msgs 
    crsf_receiver_msg
)

foreach(PACKAGE IN ITEMS ${PACKAGE_DEPENDENCIES})
  find_package(${PACKAGE} REQUIRED)
endforeach()

include_directories(include)

add_executable(rover_crfs_teleop_node
               src/rover_crfs_teleop_node.cpp 
               src/rover_crfs_teleop_controller.cpp
)

target_include_directories(rover_crfs_teleop_node PUBLIC 
                           ${CMAKE_INSTALL_PREFIX}/include
)

ament_target_dependencies(rover_crfs_teleop_node ${PACKAGE_DEPENDENCIES})

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

install(TARGETS rover_crfs_teleop_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(${PACKAGE_DEPENDENCIES})

ament_package()