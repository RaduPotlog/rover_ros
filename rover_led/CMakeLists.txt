cmake_minimum_required(VERSION 3.8)
project(rover_led)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(PACKAGE_DEPENDENCIES
    ament_cmake
    ament_index_cpp
    diagnostic_updater
    generate_parameter_library
    image_transport
    udp_msgs
    rover_msgs
    rover_utils
    pluginlib
    rclcpp
    rclcpp_components
    sensor_msgs
    std_srvs
    yaml-cpp
)

foreach(PACKAGE IN ITEMS ${PACKAGE_DEPENDENCIES})
  find_package(${PACKAGE} REQUIRED)
endforeach()

include_directories(include)

pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins.xml)

add_library(animation_plugins SHARED src/animation/image_animation.cpp
                                     src/animation/moving_image_animation.cpp)

ament_target_dependencies(animation_plugins rover_utils pluginlib rclcpp)
                                     
target_link_libraries(animation_plugins png yaml-cpp)

add_library(led_driver_node_component SHARED            
                src/led_driver_node.cpp
                src/sk9822.cpp)
                                   
ament_target_dependencies(
    led_driver_node_component
    diagnostic_updater
    image_transport
    udp_msgs
    rover_msgs
    rclcpp
    rclcpp_components
    sensor_msgs
    std_srvs
)

generate_parameter_library(led_driver_parameters
                           src/led_driver_parameters.yaml
)

target_link_libraries(led_driver_node_component
                        led_driver_parameters)

add_library(
    led_controller_node_component SHARED
    src/led_controller_node.cpp
    src/led_components/led_segment.cpp
    src/led_components/led_panel.cpp
    src/led_components/segment_converter.cpp
    src/led_components/segment_layer.cpp
    src/led_components/segment_queue_layer.cpp
)

ament_target_dependencies(
    led_controller_node_component
    rover_msgs
    rover_utils
    pluginlib
    rclcpp
    rclcpp_components
    sensor_msgs
)

generate_parameter_library(led_controller_parameters
                           src/led_controller_parameters.yaml)

target_link_libraries(led_controller_node_component yaml-cpp
                      led_controller_parameters
)

rclcpp_components_register_node(
    led_driver_node_component PLUGIN 
    "rover_led::LedDriverNode" EXECUTABLE led_driver_node
)

rclcpp_components_register_node(
    led_controller_node_component PLUGIN
    "rover_led::LedControllerNode" EXECUTABLE led_controller_node
)

install(
    TARGETS animation_plugins led_driver_node_component
            led_driver_parameters led_controller_node_component
            led_controller_parameters
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY animations config launch
        DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

ament_export_dependencies(${PACKAGE_DEPENDENCIES})
ament_export_include_directories(include/${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins.xml)

ament_package()
