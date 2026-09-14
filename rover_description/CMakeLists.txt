cmake_minimum_required(VERSION 3.10)
project(rover_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch meshes rviz urdf
        DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/setup_envs.sh.in")

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_wheel_geometry test/test_wheel_geometry.py)
endif()

ament_package()