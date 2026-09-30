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
find_package(diagnostic_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
# Interfaces-only package (no dependency on the planner): GraspCommand for prodmp_gazebo_executor_node.
find_package(satellite_grasp_msgs REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME}_core
  src/core/demonstration_recorder.cpp
  src/core/dmp.cpp
  src/core/prodmp.cpp
  src/core/quaternion_dmp.cpp
  src/core/dmp_io.cpp
  src/core/prodmp_io.cpp
  src/core/demo_csv_io.cpp
  src/core/grasp_state_machine.cpp
  src/core/cube_satellite_model.cpp
  src/core/joint_states_csv_io.cpp
  src/core/demo_params.cpp
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

# ---- ProDMP Gazebo executor node (twin of dmp_gazebo_executor_node; replays
#      an integral-form core::ProDMP for position + core::QuaternionDMP for
#      orientation) ----
add_executable(prodmp_gazebo_executor_node
  src/ros/prodmp_gazebo_executor_node.cpp
  src/ros/prodmp_executor_main.cpp
)
target_link_libraries(prodmp_gazebo_executor_node
  ${PROJECT_NAME}_core
)
ament_target_dependencies(prodmp_gazebo_executor_node
  rclcpp
  geometry_msgs
  std_msgs
  nav_msgs
  diagnostic_msgs
  tf2_ros
  ament_index_cpp
  satellite_grasp_msgs
)

# ---- ProDMP offline fit tool (STANDALONE, no ROS - links only the core
#      library, which carries Eigen3 + yaml-cpp) ----
add_executable(fit_prodmp
  src/tools/fit_prodmp_main.cpp
)
target_link_libraries(fit_prodmp
  ${PROJECT_NAME}_core
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

# ${PROJECT_NAME}_core is installed separately (top-level lib/, EXPORT set)
# from the executables below (lib/${PROJECT_NAME}/, ros2-run convention), so
# it can be consumed as an ordinary ament_cmake dependency by other colcon
# packages (e.g. satellite_grasp_planner, for core::ProDMP) via
# ament_target_dependencies(). This does not change any existing behaviour:
# the executables keep installing exactly where they did before.
install(TARGETS
  ${PROJECT_NAME}_core
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  haptic_dmp_wrapper_node
  dmp_gazebo_executor_node
  prodmp_gazebo_executor_node
  fit_prodmp
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

  ament_add_gtest(test_prodmp test/test_prodmp.cpp)
  target_link_libraries(test_prodmp ${PROJECT_NAME}_core)

  ament_add_gtest(test_quaternion_dmp test/test_quaternion_dmp.cpp)
  target_link_libraries(test_quaternion_dmp ${PROJECT_NAME}_core)

  ament_add_gtest(test_dmp_io test/test_dmp_io.cpp)
  target_link_libraries(test_dmp_io ${PROJECT_NAME}_core)

  ament_add_gtest(test_demo_csv_io test/test_demo_csv_io.cpp)
  target_link_libraries(test_demo_csv_io ${PROJECT_NAME}_core)

  ament_add_gtest(test_joint_states_csv_io test/test_joint_states_csv_io.cpp)
  target_link_libraries(test_joint_states_csv_io ${PROJECT_NAME}_core)

  ament_add_gtest(test_demo_params test/test_demo_params.cpp)
  target_link_libraries(test_demo_params ${PROJECT_NAME}_core)

  ament_add_gtest(test_grasp_state_machine test/test_grasp_state_machine.cpp)
  target_link_libraries(test_grasp_state_machine ${PROJECT_NAME}_core)

  ament_add_gtest(test_frame_correction test/test_frame_correction.cpp)
  target_link_libraries(test_frame_correction ${PROJECT_NAME}_core)

  ament_add_gtest(test_gripper_ramp test/test_gripper_ramp.cpp)
  target_link_libraries(test_gripper_ramp ${PROJECT_NAME}_core)

  ament_add_gtest(test_velocity_feedforward test/test_velocity_feedforward.cpp)
  target_link_libraries(test_velocity_feedforward ${PROJECT_NAME}_core)

  # Pure logic of satellite_rotation_mode="continuous" (header-only, no ROS).
  ament_add_gtest(test_satellite_intercept test/test_satellite_intercept.cpp)
  target_link_libraries(test_satellite_intercept ${PROJECT_NAME}_core)

  # GraspCommand -> GraspCommandInputs conversion (message types only, no node graph).
  ament_add_gtest(test_grasp_command_input test/test_grasp_command_input.cpp)
  target_link_libraries(test_grasp_command_input ${PROJECT_NAME}_core)
  ament_target_dependencies(test_grasp_command_input rclcpp satellite_grasp_msgs)

  # Pure geometry of the cube satellite model (no ROS).
  ament_add_gtest(test_cube_satellite_model test/test_cube_satellite_model.cpp)
  target_link_libraries(test_cube_satellite_model ${PROJECT_NAME}_core)

  ament_add_gtest(test_grasp_roll test/test_grasp_roll.cpp)
  target_link_libraries(test_grasp_roll ${PROJECT_NAME}_core)

  ament_add_gtest(test_grasp_cost test/test_grasp_cost.cpp)
  target_link_libraries(test_grasp_cost ${PROJECT_NAME}_core)

  # Pure logic of syncControllerAlignmentOverride()'s parameter payload (header-only, no ROS
  # node graph needed - the actual parameter-client call is not unit-testable in isolation).
  ament_add_gtest(test_controller_param_sync test/test_controller_param_sync.cpp)
  target_link_libraries(test_controller_param_sync ${PROJECT_NAME}_core)

  # Regression test for the rclcpp::SyncParametersClient / executor conflict that
  # ensureControllerParamClient() works around with a dedicated helper node (see
  # prodmp_gazebo_executor_node.cpp) - needs a live rclcpp graph (rclcpp::init/shutdown), unlike
  # the header-only tests above.
  ament_add_gtest(test_param_client_executor_regression test/test_param_client_executor_regression.cpp)
  ament_target_dependencies(test_param_client_executor_regression rclcpp)

  # Constructor-level parameter tests of prodmp_gazebo_executor_node (the node's own source is
  # compiled in; its main() lives in prodmp_executor_main.cpp, which is NOT). No spin, no Gazebo.
  ament_add_gtest(test_prodmp_executor_node_params
    test/test_prodmp_executor_node_params.cpp
    src/ros/prodmp_gazebo_executor_node.cpp)
  target_link_libraries(test_prodmp_executor_node_params ${PROJECT_NAME}_core)
  ament_target_dependencies(test_prodmp_executor_node_params
    rclcpp geometry_msgs std_msgs nav_msgs diagnostic_msgs tf2_ros ament_index_cpp satellite_grasp_msgs)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(Eigen3 yaml-cpp)

ament_package()
