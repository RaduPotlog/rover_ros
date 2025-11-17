cmake_minimum_required(VERSION 3.8)
project(rover_diag_manager)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wold-style-cast)
endif()

set(PACKAGE_DEPENDENCIES
    ament_cmake
    diagnostic_msgs
    diagnostic_updater
    generate_parameter_library
    rover_msgs
    PkgConfig
    rclcpp
    std_msgs
)

foreach(PACKAGE IN ITEMS ${PACKAGE_DEPENDENCIES})
  find_package(${PACKAGE} REQUIRED)
endforeach()

include_directories(include)

set(CPPUPROFILE_PREFIX ${CMAKE_BINARY_DIR}/ep_cppuprofile/src/ep_cppuprofile)
set(ENV{PKG_CONFIG_PATH} "${CPPUPROFILE_PREFIX}/lib:$ENV{PKG_CONFIG_PATH}")

pkg_check_modules(CPPUPROFILE REQUIRED IMPORTED_TARGET cppuprofile)

generate_parameter_library(system_diag_params
                           src/system_diag_params.yaml
)

add_executable(system_diag_node 
               src/system_diag.cpp 
               src/system_diag/system_diag_node.cpp
)

target_include_directories(system_diag_node PUBLIC 
                           ${CMAKE_INSTALL_PREFIX}/include
)

ament_target_dependencies(system_diag_node ${PACKAGE_DEPENDENCIES})
target_link_libraries(system_diag_node system_diag_params
                      PkgConfig::CPPUPROFILE
)

install(TARGETS system_diag_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(${PACKAGE_DEPENDENCIES})

ament_package()

