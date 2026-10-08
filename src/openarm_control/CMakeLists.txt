cmake_minimum_required(VERSION 3.16)
project(openarm_control LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(ct_core REQUIRED)        # control-toolbox core (ros2/src/control_toolbox, plain CMake)
find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)          # header-only, used by ct_core's integrators

# Library only: no ROS node, no executable. Nodes link it and call it.
add_library(${PROJECT_NAME} SHARED src/openarm_dynamics.cpp src/tracking_controller.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} PUBLIC ct_core pinocchio::pinocchio Eigen3::Eigen Boost::boost)

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})  # official OpenArm v1 bimanual URDF

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(ct_core pinocchio Eigen3 Boost)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_index_cpp REQUIRED)
  ament_add_gtest(test_tracking_controller test/test_tracking_controller.cpp)
  target_link_libraries(test_tracking_controller ${PROJECT_NAME})
  ament_target_dependencies(test_tracking_controller ament_index_cpp)
  target_compile_definitions(test_tracking_controller PRIVATE
    GOLDEN_CSV="${CMAKE_CURRENT_SOURCE_DIR}/test/data/golden_vectors.csv")
endif()

ament_package()
