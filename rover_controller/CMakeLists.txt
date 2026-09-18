cmake_minimum_required(VERSION 3.10)
project(rover_controller)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

# Drive-train tuning tools: `ros2 run rover_controller <tool>`.
install(PROGRAMS rover_controller/wheel_step_response.py
        DESTINATION lib/${PROJECT_NAME} RENAME wheel_step_response)
install(PROGRAMS rover_controller/wheel_odom_calibration.py
        DESTINATION lib/${PROJECT_NAME} RENAME wheel_odom_calibration)

install(DIRECTORY config launch 
        DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_wheel_geometry test/test_wheel_geometry.py)
  ament_add_pytest_test(test_response_analysis test/test_response_analysis.py)
  ament_add_pytest_test(test_calibration_analysis test/test_calibration_analysis.py)
  ament_add_pytest_test(test_controller_launch test/test_controller_launch.py
    TIMEOUT 120
    ENV ROS_LOG_DIR=/tmp/rover-controller-test-logs
  )
endif()

ament_package()
