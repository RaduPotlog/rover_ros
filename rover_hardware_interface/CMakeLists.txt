cmake_minimum_required(VERSION 3.10)
project(rover_hardware_interface)

# Use Boost's package configuration for realtime_tools dependencies.
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()


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
    rover_modbus_driver
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
# it isn't exported as an ament dependency (it has no package.xml entry).
find_package(PkgConfig REQUIRED)

include_directories(include)

generate_parameter_library(phidgets_spatial_parameters
  src/phidgets_spatial_parameters.yaml)

set(ENV{PKG_CONFIG_PATH}
    "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig:$ENV{PKG_CONFIG_PATH}")

# ROS-free core: the domain + application layers (see .claude/rules/clean_architecture.md), plus
# utils.cpp, which is std-only and which domain/driver_data_snapshot.hpp depends on. Deliberately
# gets no ROS/vendor links, so a ROS, hardware_interface or vendor-SDK
# include in domain/ or application/ now fails to compile, not just the purity scripts below.
# STATIC + PIC so it can be linked into the SHARED plugin library.
add_library(
    ${PROJECT_NAME}_core STATIC
    src/domain/driver_data_snapshot.cpp
    src/domain/rover_error_filter.cpp
    src/domain/imu_calibration.cpp
    src/domain/emergency_stop.cpp
    src/domain/velocity_command_guard.cpp
    src/application/rover_control_loop_use_case.cpp
    src/utils.cpp
)

set_target_properties(${PROJECT_NAME}_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(
    ${PROJECT_NAME}_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

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
    src/rover_safety_controller/rover_safety_controller.cpp
    src/rover_safety_controller/rover_safety_controller_e_stop_io.cpp
    src/rover_safety_controller/rover_safety_controller_gpio_adapter.cpp
)

# Keyword signatures on both calls (CMake forbids mixing plain and keyword target_link_libraries
# on one target). ${PROJECT_NAME}_core and phidgets_spatial_parameters are PUBLIC: public headers
# include their headers (e.g. rover_system.hpp -> application/, phidget_imu_sensor.hpp ->
# phidgets_spatial_parameters.hpp), and both are in this package's own export set, so exporting
# them is safe. rover_modbus_driver is PUBLIC, and must be: it is now a proper
# ament_export_dependencies() entry (the Modbus client moved to
# rover_transport/rover_modbus_driver), and this package's public headers reach into it -
# rover_a1_system.hpp includes client_settings.hpp, rover_safety_controller.hpp includes
# discrete_io_port.hpp. Downstream consumers therefore get its headers and libraries from the
# export set rather than having to find them on a default search path, which is exactly what
# the old PRIVATE Modbus_Core link could not offer.
target_link_libraries(${PROJECT_NAME} PUBLIC
    ${diagnostic_updater_TARGETS}
    ${geometry_msgs_TARGETS}
    ${hardware_interface_TARGETS}
    ${rclcpp_components_TARGETS}
    ${imu_filter_madgwick_TARGETS}
    ${rover_msgs_TARGETS}
    ${phidgets_api_TARGETS}
    ${pluginlib_TARGETS}
    ${rclcpp_TARGETS}
    ${rclcpp_lifecycle_TARGETS}
    ${realtime_tools_TARGETS}
    ${std_msgs_TARGETS}
    ${std_srvs_TARGETS}
    ${tf2_geometry_msgs_TARGETS}
    ${tf2_ros_TARGETS}
)

target_link_libraries(
    ${PROJECT_NAME}
    PUBLIC ${PROJECT_NAME}_core phidgets_spatial_parameters rover_modbus_driver::rover_modbus_driver_ros
)

target_compile_definitions(
    ${PROJECT_NAME} PRIVATE "ROVER_HARDWARE_INTERFACES_BUILDING_DLL"
)

install(
    TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_core phidgets_spatial_parameters
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
  # Standard ament style/lint gate (cpplint, cppcheck, lint_cmake, xmllint, copyright, ...),
  # matching this template's usual convention (see .claude/rules/ros2_general.md). uncrustify is
  # deliberately excluded: this repo consistently uses 4-space indentation, not
  # ament_cmake_uncrustify's default Google C++ style, so enabling it would fail every file on
  # indentation alone instead of surfacing real issues - AMENT_LINT_AUTO_EXCLUDE is the officially
  # supported way to drop one linter from the auto-discovered set (see
  # ament_lint_auto_package_hook.cmake). Kept separate from the domain-purity check below, which
  # enforces an architectural rule ament's linters don't know about.
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Domain code (include/rover_hardware_interface/domain/, src/domain/) must stay free of
  # ROS/hardware_interface/vendor-SDK dependencies. This mechanically enforces that rule so a
  # future change can't silently reintroduce one. Plain CTest add_test (pass/fail via exit code)
  # rather than ament_add_test, since the script has no JUnit-XML result-file contract to satisfy.
  enable_testing()
  add_test(
    NAME domain_purity_check
    COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/scripts/check_domain_purity.sh ${CMAKE_CURRENT_SOURCE_DIR}
  )

  # Application code (include/rover_hardware_interface/application/, src/application/) must stay
  # free of ROS/hardware_interface/vendor-SDK dependencies and may depend only on domain/ (see
  # .claude/rules/clean_architecture.md). Mirrors domain_purity_check above -
  # see scripts/check_application_purity.sh.
  add_test(
    NAME application_purity_check
    COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/scripts/check_application_purity.sh ${CMAKE_CURRENT_SOURCE_DIR}
  )

  # RoverSystem::read()/write() run on controller_manager's RT thread and must never call a
  # blocking Modbus/Phidget-SDK symbol directly (see .claude/rules/ros2_control_architecture.md
  # §5). Mechanically enforces that so a future change can't silently reintroduce a blocking call
  # on the RT path - see scripts/check_rt_path_purity.sh for the scan's scope and limits.
  add_test(
    NAME rt_path_purity_check
    COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/scripts/check_rt_path_purity.sh ${CMAKE_CURRENT_SOURCE_DIR}
  )

  # Behavioral unit tests for the pure, ROS-free domain + application layers - linked against
  # ${PROJECT_NAME}_core only, so they build and run without any ROS/vendor library (see
  # add_library() above).
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_rover_error_filter test/domain/test_rover_error_filter.cpp)
  target_link_libraries(test_rover_error_filter ${PROJECT_NAME}_core)

  ament_add_gtest(test_driver_data_snapshot test/domain/test_driver_data_snapshot.cpp)
  target_link_libraries(test_driver_data_snapshot ${PROJECT_NAME}_core)

  ament_add_gtest(test_imu_calibration test/domain/test_imu_calibration.cpp)
  target_link_libraries(test_imu_calibration ${PROJECT_NAME}_core)

  ament_add_gtest(test_emergency_stop test/domain/test_emergency_stop.cpp)
  target_link_libraries(test_emergency_stop ${PROJECT_NAME}_core)

  ament_add_gtest(
    test_velocity_command_guard test/domain/test_velocity_command_guard.cpp)
  target_link_libraries(test_velocity_command_guard ${PROJECT_NAME}_core)

  # Behavioral unit tests for the application-layer RT-loop decision logic (E-Stop state,
  # fault-flag reset, write-command gating/serialization), driven with fake domain ports the same
  # way test_emergency_stop above is - no ROS, no real hardware.
  ament_add_gtest(
    test_rover_control_loop_use_case test/application/test_rover_control_loop_use_case.cpp)
  target_link_libraries(test_rover_control_loop_use_case ${PROJECT_NAME}_core)
  # Shared test doubles (FakeRoverDriver/FakeEmergencyStop) live in test/fakes/.
  target_include_directories(test_rover_control_loop_use_case PRIVATE test)

  ament_add_gtest(test_utils test/test_utils.cpp)
  target_link_libraries(test_utils ${PROJECT_NAME}_core)

  ament_add_gtest(
    test_phidget_data_transformer test/rover_driver/test_phidget_data_transformer.cpp)
  target_link_libraries(test_phidget_data_transformer ${PROJECT_NAME})

  # Pure-logic unit test for PhidgetMotorDriver::isFailsafeTrippedReturnCode() - no live Phidget
  # handle needed (see the failsafe-watchdog design note in domain/driver.hpp's
  # MotorDriverInterface).
  ament_add_gtest(
    test_phidget_motor_driver test/rover_driver/test_phidget_motor_driver.cpp)
  target_link_libraries(test_phidget_motor_driver ${PROJECT_NAME})

  # Integration test: drives SystemROSInterface (services + publishers) over the real rclcpp
  # graph - no Phidget/Modbus hardware involved (see
  # test/system_ros_interface/test_system_ros_interface.cpp). Filed under test/system_ros_interface/
  # to mirror include/rover_hardware_interface/system_ros_interface/ - this package has no
  # production infrastructure/ directory (see the note atop check_domain_purity.sh for why).
  ament_add_gtest(
    test_system_ros_interface test/system_ros_interface/test_system_ros_interface.cpp)
  target_link_libraries(test_system_ros_interface ${PROJECT_NAME})
  target_link_libraries(test_system_ros_interface
      ${rclcpp_TARGETS}
      ${std_srvs_TARGETS}
  )

  # Integration test: drives RoverA1System through the real hardware_interface plugin contract
  # (on_init() + export_state_interfaces()/export_command_interfaces()) using hand-built
  # HardwareInfo/HardwareComponentInterfaceParams, the same types ros2_control's ResourceManager
  # populates from a parsed URDF. Deliberately does not exercise on_configure()/on_activate()/
  # read()/write(), which require real Modbus/Phidget hardware - see the file-level comment in
  # test/rover_system/test_rover_a1_system_on_init.cpp.
  ament_add_gtest(
    test_rover_a1_system_on_init test/rover_system/test_rover_a1_system_on_init.cpp)
  target_link_libraries(test_rover_a1_system_on_init ${PROJECT_NAME})
  target_link_libraries(test_rover_a1_system_on_init
      ${hardware_interface_TARGETS}
  )

  # Drives RoverSystem::write() through on_init()/on_configure() on a RoverSystem subclass whose
  # defineRoverDriver()/defineRoverController() return in-memory fakes - no Modbus/Phidget
  # hardware, unlike test_rover_a1_system_on_init above (see the test's file-level comment).
  ament_add_gtest(
    test_rover_system_write test/rover_system/test_rover_system_write.cpp)
  target_link_libraries(test_rover_system_write ${PROJECT_NAME})
  target_include_directories(test_rover_system_write PRIVATE test)
  target_link_libraries(test_rover_system_write
      ${hardware_interface_TARGETS}
      ${rclcpp_TARGETS}
      ${rclcpp_lifecycle_TARGETS}
  )

  # Integration test: drives PhidgetImuSensor through the real hardware_interface plugin contract
  # (on_init() + export_state_interfaces()), the same pattern as
  # test_rover_a1_system_on_init above. Deliberately does not exercise on_configure()/
  # on_activate()/read(), which require real Phidget Spatial USB hardware - see the file-level
  # comment in test/rover_sensors/test_phidget_imu_sensor_on_init.cpp.
  ament_add_gtest(
    test_phidget_imu_sensor_on_init test/rover_sensors/test_phidget_imu_sensor_on_init.cpp)
  target_link_libraries(test_phidget_imu_sensor_on_init ${PROJECT_NAME})
  target_link_libraries(test_phidget_imu_sensor_on_init
      ${hardware_interface_TARGETS}
  )

  # Unit tests for ContactCoilHandler/RoverSafetyController against FakeRoverModbus (test/
  # rover_safety_controller/fake_rover_modbus.hpp) - no real Modbus TCP connection involved. These
  # were previously the one infrastructure adapter in this package with no direct test coverage.
  ament_add_gtest(
    test_contact_coil_handler test/rover_safety_controller/test_contact_coil_handler.cpp)
  target_link_libraries(test_contact_coil_handler ${PROJECT_NAME})

  ament_add_gtest(
    test_rover_safety_controller test/rover_safety_controller/test_rover_safety_controller.cpp)
  target_link_libraries(test_rover_safety_controller ${PROJECT_NAME})

  # The Modbus client's own tests (connection retry, wire encoding, and a real round trip
  # against MB::TCP::Server) moved with the code to rover_transport/rover_modbus_driver.
endif()

ament_package()