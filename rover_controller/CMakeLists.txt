cmake_minimum_required(VERSION 3.10)
project(rover_controller)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch 
        DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_wheel_geometry test/test_wheel_geometry.py)
  ament_add_pytest_test(test_controller_launch test/test_controller_launch.py
    TIMEOUT 120
    ENV ROS_LOG_DIR=/tmp/rover-controller-test-logs
  )
endif()

ament_package()
