cmake_minimum_required(VERSION 3.8)
project(haptic_dmp_learning)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME}_core
  src/core/demonstration_recorder.cpp
  src/core/dmp.cpp
  src/core/quaternion_dmp.cpp
  src/core/dmp_io.cpp
  src/core/demo_csv_io.cpp
  src/core/grasp_state_machine.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_core
  Eigen3::Eigen
  yaml-cpp
)

# ---- ROS2 wrapper node executable ----
add_executable(haptic_dmp_wrapper_node
  src/ros/haptic_dmp_wrapper_node.cpp
  src/ros/main.cpp
)
target_link_libraries(haptic_dmp_wrapper_node
  ${PROJECT_NAME}_core
)
ament_target_dependencies(haptic_dmp_wrapper_node
  rclcpp
  geometry_msgs
  sensor_msgs
  ament_index_cpp
)

# ---- DMP Gazebo executor node executable ----
add_executable(dmp_gazebo_executor_node
  src/ros/dmp_gazebo_executor_node.cpp
  src/ros/dmp_executor_main.cpp
)
target_link_libraries(dmp_gazebo_executor_node
  ${PROJECT_NAME}_core
)
ament_target_dependencies(dmp_gazebo_executor_node
  rclcpp
  geometry_msgs
  std_msgs
  nav_msgs
  tf2_ros
  ament_index_cpp
)

# ---- CSV master pose player node executable (stand-in for the Geomagic
#      Touch driver while no physical device is attached) ----
add_executable(csv_master_pose_player_node
  src/ros/csv_master_pose_player_node.cpp
  src/ros/csv_master_pose_player_main.cpp
)
target_link_libraries(csv_master_pose_player_node
  ${PROJECT_NAME}_core
)
ament_target_dependencies(csv_master_pose_player_node
  rclcpp
  geometry_msgs
  sensor_msgs
  ament_index_cpp
)

# ---- Live demo recorder node executable (downstream: visualize + learn) ----
add_executable(live_demo_recorder_node
  src/ros/live_demo_recorder_node.cpp
  src/ros/live_demo_recorder_main.cpp
)
target_link_libraries(live_demo_recorder_node
  ${PROJECT_NAME}_core
)
ament_target_dependencies(live_demo_recorder_node
  rclcpp
  geometry_msgs
  sensor_msgs
  ament_index_cpp
)

# ---- Per-demo contact-force calibration / verification node ----
add_executable(grasp_force_calibration_node
  src/ros/grasp_force_calibration_node.cpp
  src/ros/grasp_force_calibration_main.cpp
)
target_link_libraries(grasp_force_calibration_node
  yaml-cpp
)
ament_target_dependencies(grasp_force_calibration_node
  rclcpp
  std_msgs
  geometry_msgs
)

# ---- Grasp state machine (fuses geometric + force-verified signals) ----
add_executable(grasp_state_machine_node
  src/ros/grasp_state_machine_node.cpp
  src/ros/grasp_state_machine_main.cpp
)
target_link_libraries(grasp_state_machine_node
  ${PROJECT_NAME}_core
)
ament_target_dependencies(grasp_state_machine_node
  rclcpp
  std_msgs
  geometry_msgs
)

# ---- Demo/replay time-sync orchestrator ----
add_executable(demo_replay_sync_orchestrator_node
  src/ros/demo_replay_sync_orchestrator_node.cpp
  src/ros/demo_replay_sync_orchestrator_main.cpp
)
ament_target_dependencies(demo_replay_sync_orchestrator_node
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  rosgraph_msgs
  ament_index_cpp
)

install(TARGETS
  ${PROJECT_NAME}_core
  haptic_dmp_wrapper_node
  dmp_gazebo_executor_node
  csv_master_pose_player_node
  live_demo_recorder_node
  grasp_force_calibration_node
  grasp_state_machine_node
  demo_replay_sync_orchestrator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_dmp test/test_dmp.cpp)
  target_link_libraries(test_dmp ${PROJECT_NAME}_core)

  ament_add_gtest(test_quaternion_dmp test/test_quaternion_dmp.cpp)
  target_link_libraries(test_quaternion_dmp ${PROJECT_NAME}_core)

  ament_add_gtest(test_dmp_io test/test_dmp_io.cpp)
  target_link_libraries(test_dmp_io ${PROJECT_NAME}_core)

  ament_add_gtest(test_demo_csv_io test/test_demo_csv_io.cpp)
  target_link_libraries(test_demo_csv_io ${PROJECT_NAME}_core)

  ament_add_gtest(test_grasp_state_machine test/test_grasp_state_machine.cpp)
  target_link_libraries(test_grasp_state_machine ${PROJECT_NAME}_core)

  ament_add_gtest(test_frame_correction test/test_frame_correction.cpp)
  target_link_libraries(test_frame_correction ${PROJECT_NAME}_core)

  ament_add_gtest(test_gripper_ramp test/test_gripper_ramp.cpp)
  target_link_libraries(test_gripper_ramp ${PROJECT_NAME}_core)
endif()

ament_package()
