cmake_minimum_required(VERSION 3.10)
project(rover_controller)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(controller_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pid_controller REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)

ament_python_install_package(${PROJECT_NAME})

# SeededPidController: pid_controller/PidController with its exported state seeded on
# activation, so the chained diff_drive never reads NaN wheel feedback on its first cycle, and
# with each wheel's command from WheelSpeedLoop (stop at zero, model-reference integral, turn
# feed-forward from diff_drive's limited body command).
add_library(${PROJECT_NAME} SHARED src/seeded_pid_controller.cpp src/wheel_speed_loop.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(${PROJECT_NAME} PUBLIC
  ${geometry_msgs_TARGETS}
  pid_controller::pid_controller
  pluginlib::pluginlib
  realtime_tools::realtime_tools)
pluginlib_export_plugin_description_file(controller_interface rover_controller_plugins.xml)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

# Drive-train tuning tools: `ros2 run rover_controller <tool>`.
install(PROGRAMS rover_controller/wheel_step_response.py
        DESTINATION lib/${PROJECT_NAME} RENAME wheel_step_response)
install(PROGRAMS rover_controller/wheel_odom_calibration.py
        DESTINATION lib/${PROJECT_NAME} RENAME wheel_odom_calibration)

install(DIRECTORY config launch 
        DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_seeded_pid_controller test/test_seeded_pid_controller.cpp)
  target_link_libraries(test_seeded_pid_controller ${PROJECT_NAME})
  find_package(control_toolbox REQUIRED)
  ament_add_gtest(test_wheel_speed_loop test/test_wheel_speed_loop.cpp)
  target_link_libraries(test_wheel_speed_loop ${PROJECT_NAME} control_toolbox::control_toolbox)

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_wheel_geometry test/test_wheel_geometry.py)
  ament_add_pytest_test(test_response_analysis test/test_response_analysis.py)
  ament_add_pytest_test(test_schedule test/test_schedule.py)
  ament_add_pytest_test(test_calibration_analysis test/test_calibration_analysis.py)
  ament_add_pytest_test(test_controller_launch test/test_controller_launch.py
    TIMEOUT 120
    ENV ROS_LOG_DIR=/tmp/rover-controller-test-logs
  )
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(controller_interface geometry_msgs pid_controller pluginlib realtime_tools)
ament_package()
