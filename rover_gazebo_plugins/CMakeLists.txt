cmake_minimum_required(VERSION 3.17)
project(rover_gazebo_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
# The gz libraries come from the ROS vendor packages, installed under /opt/ros/<distro>/opt.
find_package(gz_common_vendor REQUIRED)
find_package(gz-common REQUIRED)
find_package(gz_gui_vendor REQUIRED)
find_package(gz-gui REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz-msgs REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED COMPONENTS register)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz_transport_vendor REQUIRED)
find_package(gz-transport REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Qt6 REQUIRED COMPONENTS Core Qml Quick)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

# gz-gui loads lib<Name>.so and its QML from qrc:/<Name>/<Name>.qml, so the library, the class
# and the QML file share one name.
add_library(RoverSafetyPanel SHARED
  src/rover_safety_panel/RoverSafetyPanel.cc
  src/rover_safety_panel/RoverSafetyPanel.hh
  src/rover_safety_panel/RoverSafetyPanel.qrc
)
target_compile_features(RoverSafetyPanel PRIVATE cxx_std_17)
target_link_libraries(RoverSafetyPanel PRIVATE
  gz-common::gz-common
  gz-gui::gz-gui
  gz-msgs::gz-msgs
  gz-plugin::gz-plugin-register
  gz-transport::gz-transport
  Qt6::Core
  Qt6::Qml
  Qt6::Quick
)

# gz-sim system plugin: shuts the rclcpp context down before the server process exits
# (see RosContextShutdown.hh).
add_library(RosContextShutdown SHARED
  src/ros_context_shutdown/RosContextShutdown.cc
)
target_compile_features(RosContextShutdown PRIVATE cxx_std_17)
target_link_libraries(RosContextShutdown PRIVATE
  gz-common::gz-common
  gz-plugin::gz-plugin-register
  gz-sim::gz-sim
  rclcpp::rclcpp
)

install(TARGETS RoverSafetyPanel RosContextShutdown
  LIBRARY DESTINATION lib
)

# Lets `gz sim` find the panel by its filename in the GUI config, and the system plugin by its
# filename in the URDF.
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/gz_gui_plugin_path.sh.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/gz_sim_system_plugin_path.sh.in")

ament_package()
