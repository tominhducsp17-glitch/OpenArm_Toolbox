cmake_minimum_required(VERSION 3.16)
project(openarm_controller LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(openarm_control REQUIRED)

# Node "controller": calls the openarm_control library (control-toolbox interface), outputs tau_ff.
add_executable(controller_node src/controller_node.cpp)
target_link_libraries(controller_node openarm_control::openarm_control)
ament_target_dependencies(controller_node rclcpp sensor_msgs ament_index_cpp)
install(TARGETS controller_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_trajectory_controller.py TIMEOUT 120)
endif()

ament_package()
