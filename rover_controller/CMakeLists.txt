cmake_minimum_required(VERSION 3.8)
project(rover_controller)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch 
        DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_wheel_geometry test/test_wheel_geometry.py)
endif()

ament_package()
