cmake_minimum_required(VERSION 3.10)
project(rover_io_context)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wold-style-cast)
endif()

find_package(ament_cmake REQUIRED)
find_package(rover_asio_cmake_module REQUIRED)
find_package(ASIO REQUIRED)

# Domain + application: the byte-stream ports and the two bridges that sit between a
# stream and a publisher. No ROS, no ASIO, no OS - this is the layer the unit tests link.
add_library(${PROJECT_NAME}_core STATIC
            src/application/inbound_byte_bridge.cpp
            src/application/outbound_byte_bridge.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
                           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                           $<INSTALL_INTERFACE:include>
)
set_target_properties(${PROJECT_NAME}_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Infrastructure: the ASIO io_service and its thread pool.
#
# Named _asio rather than the house _ros because this package deliberately carries no ROS
# dependency at all - the drivers that link it do. Calling it _ros would be a lie.
add_library(${PROJECT_NAME}_asio STATIC
            src/infrastructure/io_context.cpp
)
target_link_libraries(${PROJECT_NAME}_asio PUBLIC ${PROJECT_NAME}_core)
target_include_directories(${PROJECT_NAME}_asio SYSTEM PUBLIC
                           $<BUILD_INTERFACE:${ASIO_INCLUDE_DIRS}>
)
target_compile_definitions(${PROJECT_NAME}_asio PUBLIC ${ASIO_DEFINITIONS})
set_target_properties(${PROJECT_NAME}_asio PROPERTIES POSITION_INDEPENDENT_CODE ON)

install(DIRECTORY include/ DESTINATION include)

install(TARGETS ${PROJECT_NAME}_core ${PROJECT_NAME}_asio
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Pure domain / application - no ROS, no ASIO.
  ament_add_gtest(test_byte_bridges test/unit/test_byte_bridges.cpp)
  target_link_libraries(test_byte_bridges ${PROJECT_NAME}_core)

  # Infrastructure: threads only, no device and no socket, so it lives in unit/ following
  # the precedent in rover_crsf_teleop.
  ament_add_gtest(test_io_context test/unit/test_io_context.cpp)
  target_link_libraries(test_io_context ${PROJECT_NAME}_asio)
endif()

ament_package()
