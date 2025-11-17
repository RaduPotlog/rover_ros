cmake_minimum_required(VERSION 3.8)
project(rover_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wold-style-cast)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "msg/DriverState.msg"
  "msg/DriverStateNamed.msg"
  "msg/RoverDriverState.msg"
  "msg/FaultFlag.msg"
  "msg/RuntimeError.msg"
  "msg/SystemStatus.msg"
  "msg/GpioState.msg"

  DEPENDENCIES
  std_msgs)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
