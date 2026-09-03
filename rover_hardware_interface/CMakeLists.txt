cmake_minimum_required(VERSION 3.8)
project(rover_hardware_interface)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wold-style-cast)
endif()

set(PACKAGE_DEPENDENCIES
    ament_cmake
    diagnostic_updater
    generate_parameter_library
    geometry_msgs
    hardware_interface
    # Not used directly by this package's code (no component registration here) - it is a pure
    # build-order dependency: it must resolve before imu_filter_madgwick, because that package's
    # exported CMake target links against rclcpp_components::component, and ament's
    # find_package(imu_filter_madgwick) expects that imported target to already exist.
    rclcpp_components
    imu_filter_madgwick
    rover_msgs
    phidgets_api
    pluginlib
    rclcpp
    rclcpp_lifecycle
    realtime_tools
    std_msgs
    std_srvs
    # Not used directly by this package's code either - same pure build-order rationale as
    # rclcpp_components above: imu_filter_madgwick's exported CMake target links against
    # tf2_geometry_msgs::tf2_geometry_msgs, so that imported target must already exist by the
    # time find_package(imu_filter_madgwick) runs, or CMake configure fails.
    tf2_geometry_msgs
    tf2_ros
)

foreach(PACKAGE IN ITEMS ${PACKAGE_DEPENDENCIES})
  find_package(${PACKAGE} REQUIRED)
endforeach()

# PkgConfig is a CMake-builtin module, not an ament package - kept out of PACKAGE_DEPENDENCIES so
# it isn't fed to ament_target_dependencies()/ament_export_dependencies() below (neither is
# meaningful for it, and it has no package.xml entry).
find_package(PkgConfig REQUIRED)

# Modbus_Core is a plain CMake package (not ament) from the sibling rover_modbus repo —
# not colcon-discoverable, so it must be built and installed (cmake --install) to a prefix on
# CMAKE_PREFIX_PATH *before* building this package. See src/rover_modbus/README.md.
find_package(Modbus_Core REQUIRED)

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
    src/domain/driver_data_snapshot.cpp
    src/domain/rover_error_filter.cpp
    src/domain/imu_calibration.cpp
    src/domain/emergency_stop.cpp
    src/rover_driver/rover_a1_driver.cpp
    src/rover_system/rover_a1_system.cpp
    src/rover_system/rover_system.cpp
    src/system_ros_interface/system_ros_interface.cpp
    src/utils.cpp
    src/rover_modbus/modbus.cpp
    src/rover_controller/rover_controller.cpp
    src/rover_controller/rover_controller_e_stop_io.cpp
    src/rover_controller/rover_controller_gpio_adapter.cpp
)

ament_target_dependencies(${PROJECT_NAME} ${PACKAGE_DEPENDENCIES})

target_link_libraries(
    ${PROJECT_NAME} phidgets_spatial_parameters
                    Modbus_Core::Modbus_Core
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

ament_export_dependencies(${PACKAGE_DEPENDENCIES})

ament_export_include_directories(include/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

pluginlib_export_plugin_description_file(hardware_interface
                                         rover_hardware_interfaces.xml
)

if(BUILD_TESTING)
  # Domain code (include/rover_hardware_interface/domain/, src/domain/) must stay free of
  # ROS/hardware_interface/vendor-SDK dependencies. This mechanically enforces that rule so a
  # future change can't silently reintroduce one. Plain CTest add_test (pass/fail via exit code)
  # rather than ament_add_test, since the script has no JUnit-XML result-file contract to satisfy.
  enable_testing()
  add_test(
    NAME domain_purity_check
    COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/scripts/check_domain_purity.sh ${CMAKE_CURRENT_SOURCE_DIR}
  )

  # Behavioral unit tests for the pure, ROS-free domain layer (linked against ${PROJECT_NAME},
  # which already compiles src/domain/*.cpp - see add_library() above).
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_rover_error_filter test/domain/test_rover_error_filter.cpp)
  target_link_libraries(test_rover_error_filter ${PROJECT_NAME})

  ament_add_gtest(test_driver_data_snapshot test/domain/test_driver_data_snapshot.cpp)
  target_link_libraries(test_driver_data_snapshot ${PROJECT_NAME})

  ament_add_gtest(test_imu_calibration test/domain/test_imu_calibration.cpp)
  target_link_libraries(test_imu_calibration ${PROJECT_NAME})

  ament_add_gtest(test_emergency_stop test/domain/test_emergency_stop.cpp)
  target_link_libraries(test_emergency_stop ${PROJECT_NAME})

  ament_add_gtest(test_utils test/test_utils.cpp)
  target_link_libraries(test_utils ${PROJECT_NAME})

  ament_add_gtest(
    test_phidget_data_transformer test/rover_driver/test_phidget_data_transformer.cpp)
  target_link_libraries(test_phidget_data_transformer ${PROJECT_NAME})

  # Integration test: drives SystemROSInterface (services + publishers) over the real rclcpp
  # graph - no Phidget/Modbus hardware involved (see test/infrastructure/test_system_ros_interface.cpp).
  ament_add_gtest(
    test_system_ros_interface test/infrastructure/test_system_ros_interface.cpp)
  target_link_libraries(test_system_ros_interface ${PROJECT_NAME})
  ament_target_dependencies(test_system_ros_interface rclcpp std_srvs)
endif()

ament_package()