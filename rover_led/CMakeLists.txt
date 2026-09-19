cmake_minimum_required(VERSION 3.10)
project(rover_led)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(PACKAGE_DEPENDENCIES
    ament_cmake
    ament_index_cpp
    diagnostic_updater
    generate_parameter_library
    lifecycle_msgs
    udp_msgs
    rover_msgs
    rover_utils
    pluginlib
    rclcpp
    rclcpp_components
    rclcpp_lifecycle
    sensor_msgs
    std_msgs
    std_srvs
    yaml-cpp
)

foreach(PACKAGE IN ITEMS ${PACKAGE_DEPENDENCIES})
  find_package(${PACKAGE} REQUIRED)
endforeach()

include_directories(include)

# --- Domain: entities, animations, SK9822 encoder, ports. No ROS. ----------

add_library(rover_led_domain SHARED
    src/domain/animation/image_animation.cpp
    src/domain/animation/moving_image_animation.cpp
    src/domain/led_components/led_panel.cpp
    src/domain/led_components/led_segment.cpp
    src/domain/led_components/segment_converter.cpp
    src/domain/led_components/segment_layer.cpp
    src/domain/led_components/segment_queue_layer.cpp
    src/domain/sk9822_frame_encoder.cpp
)

target_include_directories(rover_led_domain PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

# rover_utils is only used for its header-only YAML helpers.
target_include_directories(rover_led_domain PUBLIC ${rover_utils_INCLUDE_DIRS})

target_link_libraries(rover_led_domain png yaml-cpp)

# --- Application: use cases. No ROS. ----------------------------------------

add_library(rover_led_application SHARED
    src/application/encode_frame_use_case.cpp
    src/application/get_led_state_use_case.cpp
    src/application/render_tick_use_case.cpp
    src/application/set_animation_use_case.cpp
    src/application/set_brightness_use_case.cpp
    src/application/stop_animation_use_case.cpp
)

target_link_libraries(rover_led_application rover_led_domain)

# --- Infrastructure: pluginlib, YAML config, ROS nodes. ----------------------

pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins.xml)

add_library(animation_plugins SHARED src/infrastructure/animation_plugins.cpp)

target_link_libraries(animation_plugins
    ${pluginlib_TARGETS}
)

target_link_libraries(animation_plugins rover_led_domain)

generate_parameter_library(led_driver_parameters
                           src/led_driver_parameters.yaml
)

add_library(led_driver_node_component SHARED
    src/infrastructure/led_driver_node.cpp
)

target_link_libraries(led_driver_node_component
    ${diagnostic_updater_TARGETS}
    ${lifecycle_msgs_TARGETS}
    ${udp_msgs_TARGETS}
    ${rover_msgs_TARGETS}
    ${rclcpp_TARGETS}
    ${rclcpp_components_TARGETS}
    ${rclcpp_lifecycle_TARGETS}
    ${sensor_msgs_TARGETS}
    ${std_msgs_TARGETS}
    ${std_srvs_TARGETS}
)

target_link_libraries(led_driver_node_component
                      rover_led_application
                      led_driver_parameters
)
target_include_directories(led_driver_node_component PUBLIC ${rover_utils_INCLUDE_DIRS})

generate_parameter_library(led_controller_parameters
                           src/led_controller_parameters.yaml)

add_library(
    led_controller_node_component SHARED
    src/infrastructure/led_controller_node.cpp
    src/infrastructure/led_controller_diagnostics.cpp
    src/infrastructure/pluginlib_animation_factory.cpp
    src/infrastructure/yaml_led_config.cpp
)

target_link_libraries(led_controller_node_component
    ${ament_index_cpp_TARGETS}
    ${diagnostic_updater_TARGETS}
    ${rover_msgs_TARGETS}
    ${pluginlib_TARGETS}
    ${rclcpp_TARGETS}
    ${rclcpp_components_TARGETS}
    ${sensor_msgs_TARGETS}
)
target_include_directories(led_controller_node_component PUBLIC ${rover_utils_INCLUDE_DIRS})

target_link_libraries(led_controller_node_component
                      rover_led_application
                      led_controller_parameters
                      yaml-cpp
)

# Declare the include path on every exported target, not only via the
# domain library it happens to link.
foreach(TARGET_NAME IN ITEMS rover_led_application animation_plugins
                             led_driver_node_component led_controller_node_component)
  target_include_directories(${TARGET_NAME} PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  )
endforeach()

rclcpp_components_register_node(
    led_driver_node_component PLUGIN
    "rover_led::LedDriverNode" EXECUTABLE led_driver_node
)

rclcpp_components_register_node(
    led_controller_node_component PLUGIN
    "rover_led::LedControllerNode" EXECUTABLE led_controller_node
)

# --- Tests -------------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)

  # Unit tests of the domain and application layers link no ROS library.
  function(rover_led_add_test name source)
    ament_add_gmock(${name} ${source})
    target_link_libraries(${name} ${ARGN})
  endfunction()

  rover_led_add_test(test_animation test/unit/domain/test_animation.cpp rover_led_domain)
  rover_led_add_test(test_image_animation test/unit/domain/test_image_animation.cpp rover_led_domain png)
  rover_led_add_test(test_led_panel test/unit/domain/test_led_panel.cpp rover_led_domain)
  rover_led_add_test(test_led_segment test/unit/domain/test_led_segment.cpp rover_led_domain)
  rover_led_add_test(test_segment_layers test/unit/domain/test_segment_layers.cpp rover_led_domain)
  rover_led_add_test(test_segment_converter test/unit/domain/test_segment_converter.cpp rover_led_domain)
  rover_led_add_test(test_sk9822_frame_encoder test/unit/domain/test_sk9822_frame_encoder.cpp rover_led_domain)

  rover_led_add_test(test_set_animation_use_case
                     test/unit/application/test_set_animation_use_case.cpp rover_led_application)
  rover_led_add_test(test_render_tick_use_case
                     test/unit/application/test_render_tick_use_case.cpp rover_led_application)
  rover_led_add_test(test_encode_frame_use_case
                     test/unit/application/test_encode_frame_use_case.cpp rover_led_application)
  rover_led_add_test(test_stop_animation_use_case
                     test/unit/application/test_stop_animation_use_case.cpp rover_led_application)
  rover_led_add_test(test_set_brightness_use_case
                     test/unit/application/test_set_brightness_use_case.cpp rover_led_application)
  rover_led_add_test(test_get_led_state_use_case
                     test/unit/application/test_get_led_state_use_case.cpp rover_led_application)

  rover_led_add_test(test_yaml_led_config
                     test/unit/infrastructure/test_yaml_led_config.cpp led_controller_node_component)
  target_compile_definitions(test_yaml_led_config PRIVATE ROVER_LED_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

  rover_led_add_test(test_shipped_blinkers
                     test/unit/infrastructure/test_shipped_blinkers.cpp led_controller_node_component)
  target_compile_definitions(test_shipped_blinkers PRIVATE ROVER_LED_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

  rover_led_add_test(test_led_controller_diagnostics
                     test/unit/infrastructure/test_led_controller_diagnostics.cpp
                     led_controller_node_component)

  rover_led_add_test(test_led_driver_node
                     test/integration/test_led_driver_node.cpp led_driver_node_component)
endif()

install(
    TARGETS rover_led_domain rover_led_application animation_plugins
            led_driver_node_component led_driver_parameters
            led_controller_node_component led_controller_parameters
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

ament_package()
