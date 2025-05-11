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

add_service_files(
  FILES
  CheckCollision.srv
  RRTPlanPath.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
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
  third_party/ompl
)

set(core_libs
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/yaml-cpp/libyaml-cpp_lin_x64.so
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/ompl/lib/libompl.so

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

add_executable(add_truss_link_node src/add_truss_link_node.cpp)
target_link_libraries(add_truss_link_node 
  ${catkin_LIBRARIES}
  ${core_libs}
  pthread
)

add_executable(ompl_rrt_connect_node src/ompl_rrt_connect_node.cpp)
target_link_libraries(ompl_rrt_connect_node 
  ${catkin_LIBRARIES}
  ${core_libs}
  pthread
)

add_executable(check_collision_server src/check_collision_server.cpp)
target_link_libraries(check_collision_server
  ${catkin_LIBRARIES}
)
add_dependencies(check_collision_server 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

add_executable(ompl_rrt_server src/ompl_rrt_server.cpp)
target_link_libraries(ompl_rrt_server 
  ${catkin_LIBRARIES}
  ${core_libs}
  pthread
)