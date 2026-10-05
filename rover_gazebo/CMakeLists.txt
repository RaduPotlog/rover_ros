cmake_minimum_required(VERSION 3.10)
project(rover_gazebo)

# Launch, config and sim helper scripts only: nothing is compiled here. The Gazebo
# ros2_control plugin comes from gz_ros2_control, the Rover Safety GUI panel from
# rover_gazebo_plugins (see package.xml).
find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
        DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/sim_safety_plc.py scripts/rover_sim.sh
        DESTINATION lib/${PROJECT_NAME}
)

# Imported by sim_safety_plc.py from its own directory.
install(FILES scripts/sim_safety_plc_model.py
        DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_sim_safety_plc_model test/test_sim_safety_plc_model.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts
  )
endif()

ament_package()
