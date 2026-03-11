cmake_minimum_required(VERSION 3.8)
project(rover_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(PACKAGE_DEPENDENCIES
    ament_cmake
    ament_cmake_python
    diagnostic_updater
    realtime_tools
    rclcpp
    std_msgs
    yaml-cpp)

foreach(PACKAGE IN ITEMS ${PACKAGE_DEPENDENCIES})
  find_package(${PACKAGE} REQUIRED)
endforeach()

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS rover_utils/__init__.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS rover_utils/logging.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS rover_utils/messages.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS rover_utils/version_check.py DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

ament_export_include_directories(include/${PROJECT_NAME})

ament_package()
