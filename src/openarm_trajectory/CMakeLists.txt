cmake_minimum_required(VERSION 3.16)
project(openarm_trajectory LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(trajectory_node src/trajectory_node.cpp)
target_link_libraries(trajectory_node Eigen3::Eigen)
ament_target_dependencies(trajectory_node rclcpp sensor_msgs std_msgs)
install(TARGETS trajectory_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY trajectories DESTINATION share/${PROJECT_NAME})

ament_package()
