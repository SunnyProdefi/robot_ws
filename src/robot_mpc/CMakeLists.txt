cmake_minimum_required(VERSION 3.0.2)
project(robot_mpc)

# -------------------- Compile Options --------------------
add_compile_options(-std=c++17)

# -------------------- Find catkin and message packages --------------------
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  roslib
  geometry_msgs
  message_generation
)

# -------------------- Service Definition --------------------
# add_service_files(
#   FILES
#   PlanPath.srv
#   CartesianInterpolation.srv
#   RobotPose.srv
# )

# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

# -------------------- catkin Package --------------------
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs roslib message_runtime geometry_msgs
  INCLUDE_DIRS include
  # DEPENDS message_generation
)

# -------------------- Include Directories --------------------
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  third_party/eigen3
  third_party/pinocchio
  third_party/quill
  third_party/qpOASES
  third_party/urdfdom
  third_party/yaml-cpp
  third_party/quill/quill/bundled
  third_party/boost
)

# -------------------- Define Core Libraries --------------------
# core 库只链接静态库
set(core_static_libs
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/pinocchio/libpinocchio_lin_x64.a
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/urdfdom/liburdfdom_model_lin_x64.a
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/urdfdom/libtinyxml_lin_x64.a
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/urdfdom/libconsole_bridge_lin_x64.a
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/quill/libquill_lin_x64.a
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/qpOASES/libqpOASES_lin_x64.a
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/yaml-cpp/libyaml-cpp_lin_x64.so
)

set(CORE_SRC
  src/mpc_controller.cpp
)

add_library(core_mpc STATIC ${CORE_SRC})
target_link_libraries(core_mpc ${core_static_libs} pthread)

# 可执行文件链接动态库
add_executable(robot_mpc src/robot_mpc.cpp)
add_dependencies(robot_mpc ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(robot_mpc
  ${catkin_LIBRARIES}
  core_mpc
)

# -------------------- End --------------------
