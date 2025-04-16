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
  third_party/coal
  third_party/eigen3
  third_party/ompl
  third_party/pinocchio
  third_party/urdfdom
  third_party/yaml-cpp
)

set(core_libs
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/coal/libcoal.so
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/ompl/lin_x64/libompl.so
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/pinocchio/libpinocchio_collision.so
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/pinocchio/libpinocchio_default.so
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/pinocchio/libpinocchio_parsers.so
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/urdfdom/liburdfdom_model_lin_x64.a
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/urdfdom/libtinyxml_lin_x64.a
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/urdfdom/libconsole_bridge_lin_x64.a
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/yaml-cpp/libyaml-cpp_lin_x64.so
)

set(CORE_SRC
  src/BSplineInterpolator.cpp
  src/robot_model.cpp
)

add_library(core_rrt STATIC ${CORE_SRC})
target_link_libraries(core_rrt ${core_libs} pthread)

add_executable(dualArmRRT src/dualArmRRT.cpp)
add_dependencies(dualArmRRT ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dualArmRRT
  ${catkin_LIBRARIES}
  core_rrt
)
