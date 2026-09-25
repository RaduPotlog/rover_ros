cmake_minimum_required(VERSION 3.10)
project(rover_modbus_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wold-style-cast)
endif()

set(PACKAGE_DEPENDENCIES
    rclcpp
    rover_modbus
)

find_package(ament_cmake REQUIRED)
foreach(PACKAGE IN ITEMS ${PACKAGE_DEPENDENCIES})
  find_package(${PACKAGE} REQUIRED)
endforeach()

# Domain + application: the ports, the settings, the DiscreteRequest/DiscreteReply transaction
# values, and the client that turns contact/coil reads and writes into those transactions. Needs
# Modbus_Core for MB::ModbusException alone - DiscreteIoPort's documented failure type - and
# NOT Modbus_Tcp, so "no sockets in _core" is enforced by the linker rather than by convention.
# scripts/check_core_purity.sh keeps every other MB:: header out of domain/ and application/.
add_library(${PROJECT_NAME}_core STATIC
  src/domain/logger_port.cpp
  src/application/modbus_discrete_io_client.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_core PUBLIC rover_modbus::Modbus_Core)
set_target_properties(${PROJECT_NAME}_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Infrastructure, socket-free and ROS-free: the only DiscreteRequest/DiscreteReply <-> MB:: frame
# translation. Its own library so the unit tests' fake transport runs the production mapping.
add_library(${PROJECT_NAME}_mb_mapping STATIC src/infrastructure/mb_frame_mapping.cpp)
target_link_libraries(${PROJECT_NAME}_mb_mapping PUBLIC ${PROJECT_NAME}_core rover_modbus::Modbus_Core)
set_target_properties(${PROJECT_NAME}_mb_mapping PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Infrastructure: the TCP socket adapter, the rclcpp logger and the factory that wires
# them to the client.
add_library(${PROJECT_NAME}_ros STATIC
  src/infrastructure/modbus_tcp_transport.cpp
  src/infrastructure/rclcpp_logger.cpp
  src/infrastructure/modbus_tcp_client_factory.cpp
)
target_link_libraries(${PROJECT_NAME}_ros PUBLIC
  ${PROJECT_NAME}_core
  ${PROJECT_NAME}_mb_mapping
  rover_modbus::Modbus_Tcp
  ${rclcpp_TARGETS}
)
set_target_properties(${PROJECT_NAME}_ros PROPERTIES POSITION_INDEPENDENT_CODE ON)

install(DIRECTORY include/ DESTINATION include)

install(TARGETS ${PROJECT_NAME}_core ${PROJECT_NAME}_mb_mapping ${PROJECT_NAME}_ros
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${PACKAGE_DEPENDENCIES})

if(BUILD_TESTING)
  # The house uses a 4-space style that uncrustify's ament profile disagrees with.
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  # Pure application logic against a fake transport - no socket, no ROS. The fake now runs the
  # production MB:: mapping, so it needs _mb_mapping too.
  ament_add_gtest(test_modbus_discrete_io_client test/unit/test_modbus_discrete_io_client.cpp)
  target_link_libraries(test_modbus_discrete_io_client ${PROJECT_NAME}_core ${PROJECT_NAME}_mb_mapping)
  target_include_directories(test_modbus_discrete_io_client PRIVATE test)

  # The same client, exercised through every failure path the fake can inject (empty/short
  # replies, non-coil cells, transport exceptions, the engage guard, the backoff window).
  ament_add_gtest(test_modbus_discrete_io_client_failures test/unit/test_modbus_discrete_io_client_failures.cpp)
  target_link_libraries(test_modbus_discrete_io_client_failures ${PROJECT_NAME}_core ${PROJECT_NAME}_mb_mapping)
  target_include_directories(test_modbus_discrete_io_client_failures PRIVATE test)

  # Integration: the connection-failure path against a real, closed loopback port.
  ament_add_gtest(test_modbus_tcp_transport test/integration/test_modbus_tcp_transport.cpp)
  target_link_libraries(test_modbus_tcp_transport ${PROJECT_NAME}_ros)

  # E2E: a real MB::TCP::Server on a pid-derived port, one full request/response exchange.
  ament_add_gtest(test_modbus_tcp_roundtrip test/e2e/test_modbus_tcp_roundtrip.cpp TIMEOUT 60)
  target_link_libraries(test_modbus_tcp_roundtrip ${PROJECT_NAME}_ros)

  # The same failure paths, but against a real scripted TCP server instead of the fake -
  # proves the golden wire frames and the reply-handling table together.
  ament_add_gtest(test_modbus_tcp_reply_handling test/e2e/test_modbus_tcp_reply_handling.cpp TIMEOUT 60)
  target_link_libraries(test_modbus_tcp_reply_handling ${PROJECT_NAME}_ros)
  target_include_directories(test_modbus_tcp_reply_handling PRIVATE test)

  # The MB:: <-> DiscreteRequest/DiscreteReply mapping alone, including byte-identity
  # against the pre-change MB::ModbusRequest construction.
  ament_add_gtest(test_mb_frame_mapping test/unit/test_mb_frame_mapping.cpp)
  target_link_libraries(test_mb_frame_mapping ${PROJECT_NAME}_mb_mapping)

  # scripts/check_core_purity.sh - keeps MB:: frame types out of domain/ and application/. A plain
  # CTest add_test (pass/fail via exit code), like rover_hardware_interface's purity checks.
  add_test(
    NAME core_purity_check
    COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/scripts/check_core_purity.sh ${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()

ament_package()
