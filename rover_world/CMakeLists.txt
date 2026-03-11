cmake_minimum_required(VERSION 3.8)
project(rover_world)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch world
        DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/setup_envs.sh.in")

ament_package()
