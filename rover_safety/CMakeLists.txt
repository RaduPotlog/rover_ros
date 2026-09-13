cmake_minimum_required(VERSION 3.10)
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

# Domain: plain C++, no ROS dependencies.
add_library(${PROJECT_NAME}_core STATIC
    src/domain/battery_safety_policy.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# ------------------------------------- Plugins -------------------------------------- #
add_library(call_set_bool_service_bt_node SHARED
    src/plugins/action/call_set_bool_service_node.cpp)
list(APPEND plugin_libs call_set_bool_service_bt_node)

add_library(call_trigger_service_bt_node SHARED
    src/plugins/action/call_trigger_service_node.cpp)
list(APPEND plugin_libs call_trigger_service_bt_node)

add_library(call_set_led_animation_service_bt_node SHARED
            src/plugins/action/call_set_led_animation_service_node.cpp)
list(APPEND plugin_libs call_set_led_animation_service_bt_node)

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
    target_link_libraries(${bt_plugin}
        ${ament_index_cpp_TARGETS}
        ${behaviortree_cpp_TARGETS}
        ${geometry_msgs_TARGETS}
        ${rover_msgs_TARGETS}
        OpenSSL::SSL
        OpenSSL::Crypto
        ${rclcpp_TARGETS}
        ${rclcpp_action_TARGETS}
        ${sensor_msgs_TARGETS}
        ${std_msgs_TARGETS}
        ${std_srvs_TARGETS}
        ${tf2_geometry_msgs_TARGETS}
        yaml-cpp
        ${nav2_behavior_tree_TARGETS}
    )
    target_include_directories(${bt_plugin} PUBLIC ${rover_utils_INCLUDE_DIRS})
endforeach()

add_executable(${PROJECT_NAME}_node src/safety_main.cpp
    src/safety_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
    ${behaviortree_cpp_TARGETS}
    ${geometry_msgs_TARGETS}
    ${rover_msgs_TARGETS}
    ${rclcpp_TARGETS}
    ${sensor_msgs_TARGETS}
    ${std_msgs_TARGETS}
    ${tf2_geometry_msgs_TARGETS}
    ${nav2_behavior_tree_TARGETS}
)
target_include_directories(${PROJECT_NAME}_node PUBLIC ${rover_utils_INCLUDE_DIRS})

generate_parameter_library(safety_parameters
    src/safety_parameters.yaml
)

target_link_libraries(${PROJECT_NAME}_node
    ${plugin_libs}
    safety_parameters
    ${PROJECT_NAME}_core
)

add_executable(led_safety_node src/led_safety_main.cpp
                                   src/led_safety_node.cpp)

target_link_libraries(led_safety_node
    ${behaviortree_cpp_TARGETS}
    ${geometry_msgs_TARGETS}
    ${rover_msgs_TARGETS}
    ${rclcpp_TARGETS}
    ${sensor_msgs_TARGETS}
    ${std_msgs_TARGETS}
    ${tf2_geometry_msgs_TARGETS}
    ${nav2_behavior_tree_TARGETS}
)
target_include_directories(led_safety_node PUBLIC ${rover_utils_INCLUDE_DIRS})

generate_parameter_library(led_safety_parameters
                           src/led_safety_parameters.yaml
)

target_link_libraries(led_safety_node ${plugin_libs}
                      led_safety_parameters
)

install(TARGETS ${plugin_libs} 
    DESTINATION lib)

install(TARGETS ${PROJECT_NAME}_node led_safety_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY behavior_trees config launch
    DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/ 
    DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)

    ament_add_gtest(test_battery_safety_policy test/unit/test_battery_safety_policy.cpp)
    target_link_libraries(test_battery_safety_policy ${PROJECT_NAME}_core)

    ament_add_gtest(test_call_trigger_service test/integration/test_call_trigger_service.cpp
                    TIMEOUT 60)
    target_link_libraries(test_call_trigger_service call_trigger_service_bt_node)
    target_link_libraries(test_call_trigger_service
        ${behaviortree_cpp_TARGETS}
        ${nav2_behavior_tree_TARGETS}
        ${rclcpp_TARGETS}
        ${std_srvs_TARGETS}
    )
endif()

ament_export_dependencies(${PACKAGE_DEPENDENCIES})
ament_export_include_directories(include/${PROJECT_NAME})

ament_package()