cmake_minimum_required(VERSION 3.10)
project(rover_utils)

# Use Boost's package configuration for realtime_tools dependencies.
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()

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

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_shutdown_gate test/test_shutdown_gate.cpp)
  target_include_directories(test_shutdown_gate PRIVATE include)
  target_link_libraries(test_shutdown_gate rclcpp::rclcpp)

  ament_add_gtest(test_yaml_utils test/test_yaml_utils.cpp)
  target_include_directories(test_yaml_utils PRIVATE include)
  target_link_libraries(test_yaml_utils yaml-cpp)

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_logging test/test_logging.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(test_events test/test_events.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(test_messages test/test_messages.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(test_shutdown test/test_shutdown.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(test_version_check test/test_version_check.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
endif()

ament_package()
