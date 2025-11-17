cmake_minimum_required(VERSION 3.8)
project(rover_hardware_interface)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wold-style-cast)
endif()

set(PACKAGE_DEPENDENCIES
    ament_cmake
    ament_index_cpp
    controller_interface
    diagnostic_updater
    generate_parameter_library
    geometry_msgs
    hardware_interface
    imu_filter_madgwick
    rover_msgs
    phidgets_api
    PkgConfig
    pluginlib
    rclcpp
    rclcpp_lifecycle
    realtime_tools
    std_srvs
    tf2_geometry_msgs
    tf2_ros
)

foreach(PACKAGE IN ITEMS ${PACKAGE_DEPENDENCIES})
  find_package(${PACKAGE} REQUIRED)
endforeach()

include_directories(include)

generate_parameter_library(phidgets_spatial_parameters
                           src/phidgets_spatial_parameters.yaml)

set(ENV{PKG_CONFIG_PATH}
    "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig:$ENV{PKG_CONFIG_PATH}")

add_library(
    ${PROJECT_NAME} SHARED
    src/rover_sensors/phidget_imu_sensor.cpp
    src/rover_driver/phidget_driver/phidget_utils.cpp
    src/rover_driver/phidget_driver/phidget_motor_driver.cpp
    src/rover_driver/phidget_driver/phidget_rover_driver.cpp
    src/rover_driver/phidget_driver/phidget_data_transformer.cpp
    src/rover_driver/rover_a1_driver.cpp
    src/rover_system/rover_a1_system.cpp
    src/rover_system/rover_system.cpp
    src/system_ros_interface/system_ros_interface.cpp
    src/system_e_stop/system_e_stop.cpp
    src/utils.cpp
    src/rover_modbus/modbus.cpp
    src/rover_controller/rover_controller.cpp
)

ament_target_dependencies(${PROJECT_NAME} ${PACKAGE_DEPENDENCIES})

target_link_libraries(
    ${PROJECT_NAME} phidgets_spatial_parameters
                    Modbus_Core
)

target_compile_definitions(
    ${PROJECT_NAME} PRIVATE "ROVER_HARDWARE_INTERFACES_BUILDING_DLL"
)

install(
    TARGETS ${PROJECT_NAME} phidgets_spatial_parameters
    EXPORT export_${PROJECT_NAME}
    RUNTIME DESTINATION bin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(${PACKAGE_DEPENDENCIES})

ament_export_include_directories(include/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

pluginlib_export_plugin_description_file(hardware_interface
                                         rover_hardware_interfaces.xml
)

ament_package()