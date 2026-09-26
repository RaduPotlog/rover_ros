cmake_minimum_required(VERSION 3.10)
project(rover_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  # foxglove_bridge's topic whitelist covers what the web UIs use, and keeps the rest off.
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_web_bridges_whitelist test/test_web_bridges_whitelist.py)
endif()

ament_package()

