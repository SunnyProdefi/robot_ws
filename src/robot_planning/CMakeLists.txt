cmake_minimum_required(VERSION 3.0.2)
project(robot_planning)

# -------------------- Compile Options --------------------
add_compile_options(-std=c++17)

# -------------------- Find catkin and message packages --------------------
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  roslib
  message_generation
)

# -------------------- Service Definition --------------------
add_service_files(
  FILES
  PlanPath.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

# -------------------- catkin Package --------------------
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs roslib message_runtime
  INCLUDE_DIRS include
)

# -------------------- Include Directories --------------------
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  third_party/eigen3
  third_party/glfw
  third_party/jsoncpp
  third_party/pinocchio
  third_party/quill
  third_party/qpOASES
  third_party/urdfdom
  third_party/yaml-cpp
  third_party/quill/quill/bundled
  third_party/boost
  third_party/mujoco
  math
)

# -------------------- Link Directories --------------------
link_directories(
  third_party/mujoco/lin_x64
  third_party/pinocchio/lin_x64
  third_party/jsoncpp/lin_x64
  third_party/urdfdom/lin_x64
  third_party/quill/lin_x64
  third_party/qpOASES/lin_x64
  third_party/yaml-cpp/lin_x64
)

# -------------------- Define Core Libraries --------------------
set(core_libs
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/pinocchio/libpinocchio_lin_x64.a
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/urdfdom/liburdfdom_model_lin_x64.a
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/urdfdom/libtinyxml_lin_x64.a
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/urdfdom/libconsole_bridge_lin_x64.a
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/jsoncpp/libjsoncpp_lin_x64.a
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/quill/libquill_lin_x64.a
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/qpOASES/libqpOASES_lin_x64.a
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/yaml-cpp/libyaml-cpp_lin_x64.so
)

set(sim_libs
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/glfw/libglfw3_lin_x64.a
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/mujoco/lin_x64/libmujoco.so
  dl
)

# -------------------- Collect Core Source Files --------------------
set(CORE_SRC
  # math/*.cpp
  src/ik_solver.cpp
  src/leg_transform_cs.cpp
  src/leg_ik_cs.cpp
  src/ikfast_single_arm.cpp
  src/ikfast_wrapper_single_arm.cpp
  src/full_body_trajectory.cpp
  src/yaml_common.cpp
)

# -------------------- Core Library --------------------
add_library(core STATIC ${CORE_SRC})
target_link_libraries(core ${core_libs} pthread)

# -------------------- ROS Executables --------------------
add_executable(robot_planning_node src/robot_planning.cpp)
add_dependencies(robot_planning_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(robot_planning_node
  ${catkin_LIBRARIES}
  core
  ${sim_libs}
)

# -------------------- Example Executable --------------------
add_executable(example_leg_transform src/example_leg_transform.cpp)
add_dependencies(example_leg_transform ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_leg_transform
  ${catkin_LIBRARIES}
  core
  ${sim_libs}
)

add_executable(get_base_link_pose src/get_base_link_pose.cpp)
add_dependencies(get_base_link_pose ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(get_base_link_pose
  ${catkin_LIBRARIES}
  core
  ${sim_libs}
)

add_executable(get_base_link_pose_test src/get_base_link_pose_test.cpp)
add_dependencies(get_base_link_pose_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(get_base_link_pose_test
  ${catkin_LIBRARIES}
  core
  ${sim_libs}
)

# -------------------- End --------------------
