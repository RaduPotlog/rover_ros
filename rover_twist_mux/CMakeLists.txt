cmake_minimum_required(VERSION 3.10)
project(rover_twist_mux)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED YES)
set(CMAKE_CXX_EXTENSIONS NO)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wold-style-cast)
endif()

set(PACKAGE_DEPENDENCIES
    generate_parameter_library
    rclcpp
    rover_msgs
    std_msgs
)

find_package(ament_cmake REQUIRED)
foreach(PACKAGE IN ITEMS ${PACKAGE_DEPENDENCIES})
  find_package(${PACKAGE} REQUIRED)
endforeach()

generate_parameter_library(motion_lock_params
                           src/motion_lock_params.yaml
)

# Domain: plain C++, no ROS dependencies.
add_library(${PROJECT_NAME}_core STATIC
            src/domain/motion_lock_policy.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
                           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# Infrastructure: the ROS 2 adapter translating gpio_state into the twist_mux lock topic.
add_library(${PROJECT_NAME}_ros STATIC
            src/infrastructure/motion_lock_node.cpp
)
target_link_libraries(${PROJECT_NAME}_ros
                      ${PROJECT_NAME}_core
                      motion_lock_params
)
target_link_libraries(${PROJECT_NAME}_ros
    ${rclcpp_TARGETS}
    ${rover_msgs_TARGETS}
    ${std_msgs_TARGETS}
)

add_executable(motion_lock_node src/motion_lock.cpp)
target_link_libraries(motion_lock_node ${PROJECT_NAME}_ros)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

install(TARGETS motion_lock_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_motion_lock_policy test/unit/test_motion_lock_policy.cpp)
  target_link_libraries(test_motion_lock_policy ${PROJECT_NAME}_core)
endif()

ament_export_dependencies(${PACKAGE_DEPENDENCIES})

ament_package()
