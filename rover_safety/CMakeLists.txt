cmake_minimum_required(VERSION 3.8)
project(rover_safety)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(PACKAGE_DEPENDENCIES
    ament_cmake
    ament_index_cpp
    behaviortree_cpp
    generate_parameter_library
    geometry_msgs
    rover_msgs
    rover_utils
    OpenSSL
    rclcpp
    rclcpp_action
    sensor_msgs
    std_msgs
    std_srvs
    tf2_geometry_msgs
    yaml-cpp
    nav2_behavior_tree
)

foreach(PACKAGE IN ITEMS ${PACKAGE_DEPENDENCIES})
    find_package(${PACKAGE} REQUIRED)
endforeach()

include_directories(include)

# ------------------------------------- Plugins -------------------------------------- #
add_library(call_set_bool_service_bt_node SHARED
    src/plugins/action/call_set_bool_service_node.cpp)
list(APPEND plugin_libs call_set_bool_service_bt_node)

add_library(call_trigger_service_bt_node SHARED
    src/plugins/action/call_trigger_service_node.cpp)
list(APPEND plugin_libs call_trigger_service_bt_node)

add_library(execute_command_bt_node SHARED
    src/plugins/action/execute_command_node.cpp)
list(APPEND plugin_libs execute_command_bt_node)

add_library(signal_shutdown_bt_node SHARED
    src/plugins/action/signal_shutdown_node.cpp)
list(APPEND plugin_libs signal_shutdown_bt_node)

add_library(tick_after_timeout_bt_node SHARED
    src/plugins/decorator/tick_after_timeout_node.cpp)
list(APPEND plugin_libs tick_after_timeout_bt_node)
# ------------------------------------- End plugins =--------------------------------- #

foreach(bt_plugin ${plugin_libs})
    target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
    ament_target_dependencies(${bt_plugin} ${PACKAGE_DEPENDENCIES})
endforeach()

add_executable(${PROJECT_NAME}_node src/safety_main.cpp
    src/safety_node.cpp
)

ament_target_dependencies(
    ${PROJECT_NAME}_node
    behaviortree_cpp
    geometry_msgs
    rover_msgs
    rover_utils
    rclcpp
    sensor_msgs
    std_msgs
    tf2_geometry_msgs
    nav2_behavior_tree
)

generate_parameter_library(safety_parameters
    src/safety_parameters.yaml
)

target_link_libraries(${PROJECT_NAME}_node 
    ${plugin_libs}
    safety_parameters
)

install(TARGETS ${plugin_libs} 
    DESTINATION lib)

install(TARGETS ${PROJECT_NAME}_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY behavior_trees config launch
    DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/ 
    DESTINATION include/${PROJECT_NAME}
)

ament_export_dependencies(${PACKAGE_DEPENDENCIES})
ament_export_include_directories(include/${PROJECT_NAME})

ament_package()