cmake_minimum_required(VERSION 3.10)
project(rover_gazebo)

# Launch, config and a sim helper script only: nothing is compiled here. The Gazebo
# ros2_control plugin comes from gz_ros2_control (see package.xml).
find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
        DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/sim_gpio_state.py scripts/rover_sim.sh
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()
