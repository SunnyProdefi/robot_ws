cmake_minimum_required(VERSION 3.0.2)
project(robot_rrt)

# -------------------- Compile Options --------------------
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  roscpp
  roslib
  std_msgs
  message_generation
  visualization_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs roslib message_runtime geometry_msgs visualization_msgs
  INCLUDE_DIRS include
  DEPENDS message_generation
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  third_party/boost
  third_party/eigen3
  third_party/yaml-cpp
)

set(core_libs
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/yaml-cpp/libyaml-cpp_lin_x64.so
)

add_executable(rrt_cli src/rrt_cli.cpp)
add_dependencies(rrt_cli ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rrt_cli
  ${catkin_LIBRARIES}
  ${core_libs}
  pthread
)

add_executable(collision_check_node src/collision_check_node.cpp)
target_link_libraries(collision_check_node 
  ${catkin_LIBRARIES}
  ${core_libs}
  pthread
)

