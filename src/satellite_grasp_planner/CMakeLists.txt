cmake_minimum_required(VERSION 3.8)
project(satellite_grasp_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(coal REQUIRED)
find_package(haptic_dmp_learning REQUIRED)
find_package(franka_cartesian_control REQUIRED)

include_directories(include)

# ---- Level 1 core library (Eigen3 + Pinocchio only, NO ROS) ----
# Depends on haptic_dmp_learning_core (core::ProDMP, for candidate rollouts)
# and franka_cartesian_control_core (core::RobotModel, core::VelocityIkSolver,
# core::computePoseError - reused verbatim for the offline joint-path
# simulation, see core/joint_path_simulator.hpp).
add_library(${PROJECT_NAME}_core
  src/core/manipulability.cpp
  src/core/joint_path_simulator.cpp
  src/core/phase_selector.cpp
  src/core/kinematic_feasibility.cpp
  src/core/satellite_collision.cpp
  src/core/candidate_scan.cpp
  src/core/candidate_eval.cpp
  src/core/grasp_selection.cpp
  src/core/launch_plan.cpp
  src/core/selection_inputs.cpp
  src/core/grasp_launch.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_core
  Eigen3::Eigen
  pinocchio::pinocchio
  coal::coal
)
# haptic_dmp_learning and franka_cartesian_control export their _core
# libraries the classic ament_cmake way (ament_export_libraries +
# ament_export_include_directories, no ament_export_targets), so they are
# consumed via ament_target_dependencies (<pkg>_INCLUDE_DIRS/<pkg>_LIBRARIES),
# not as namespaced CMake targets.
ament_target_dependencies(${PROJECT_NAME}_core
  haptic_dmp_learning
  franka_cartesian_control
)

install(TARGETS
  ${PROJECT_NAME}_core
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

# ---- ROS2 wrapper: one-shot grasp planner node (Task 1 of 2 - see DESIGN_NOTES.md) ----
# grasp_planner_ros_lib is split from grasp_planner_node/main so the pure validation
# (grasp_planner_checks) and the message conversion (grasp_command_conversion) can be linked into
# tests directly, without needing rclcpp::init(). ${PROJECT_NAME}_core itself stays rclcpp-free.
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(satellite_grasp_msgs REQUIRED)

add_library(grasp_planner_ros_lib
  src/ros/grasp_planner_checks.cpp
  src/ros/grasp_command_conversion.cpp
)
target_link_libraries(grasp_planner_ros_lib
  Eigen3::Eigen
  pinocchio::pinocchio
  coal::coal
)
# grasp_command_conversion.cpp includes grasp_launch.hpp, which transitively pulls in
# haptic_dmp_learning/franka_cartesian_control headers (CubeSatelliteModel, RobotModel, ...) - needed
# even though this .cpp never calls into them, matching every other target in this file that includes
# those headers.
ament_target_dependencies(grasp_planner_ros_lib
  rclcpp
  satellite_grasp_msgs
  haptic_dmp_learning
  franka_cartesian_control
)

add_executable(grasp_planner_node
  src/ros/grasp_planner_node.cpp
  src/ros/grasp_planner_main.cpp
)
target_link_libraries(grasp_planner_node
  grasp_planner_ros_lib
  ${PROJECT_NAME}_core
  pinocchio::pinocchio
  coal::coal
)
ament_target_dependencies(grasp_planner_node
  rclcpp
  nav_msgs
  satellite_grasp_msgs
  haptic_dmp_learning
  franka_cartesian_control
)

install(TARGETS grasp_planner_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_phase_selector test/test_phase_selector.cpp)
  target_link_libraries(test_phase_selector
    ${PROJECT_NAME}_core
    pinocchio::pinocchio
  )
  ament_target_dependencies(test_phase_selector
    haptic_dmp_learning
    franka_cartesian_control
  )

  ament_add_gtest(test_kinematic_feasibility test/test_kinematic_feasibility.cpp)
  target_link_libraries(test_kinematic_feasibility
    ${PROJECT_NAME}_core
    pinocchio::pinocchio
  )
  ament_target_dependencies(test_kinematic_feasibility
    haptic_dmp_learning
    franka_cartesian_control
  )

  ament_add_gtest(test_selection_inputs test/test_selection_inputs.cpp)
  target_link_libraries(test_selection_inputs
    ${PROJECT_NAME}_core
    pinocchio::pinocchio
  )
  ament_target_dependencies(test_selection_inputs
    haptic_dmp_learning
    franka_cartesian_control
  )

  ament_add_gtest(test_launch_plan test/test_launch_plan.cpp)
  target_link_libraries(test_launch_plan
    ${PROJECT_NAME}_core
    pinocchio::pinocchio
  )
  ament_target_dependencies(test_launch_plan
    haptic_dmp_learning
    franka_cartesian_control
  )

  ament_add_gtest(test_grasp_selection test/test_grasp_selection.cpp)
  target_link_libraries(test_grasp_selection
    ${PROJECT_NAME}_core
    pinocchio::pinocchio
  )
  ament_target_dependencies(test_grasp_selection
    haptic_dmp_learning
    franka_cartesian_control
  )

  ament_add_gtest(test_candidate_eval test/test_candidate_eval.cpp)
  target_link_libraries(test_candidate_eval
    ${PROJECT_NAME}_core
    pinocchio::pinocchio
  )
  ament_target_dependencies(test_candidate_eval
    haptic_dmp_learning
    franka_cartesian_control
  )

  ament_add_gtest(test_candidate_scan test/test_candidate_scan.cpp)
  target_link_libraries(test_candidate_scan
    ${PROJECT_NAME}_core
    pinocchio::pinocchio
  )
  ament_target_dependencies(test_candidate_scan
    haptic_dmp_learning
    franka_cartesian_control
  )

  ament_add_gtest(test_satellite_collision test/test_satellite_collision.cpp)
  target_link_libraries(test_satellite_collision
    ${PROJECT_NAME}_core
    pinocchio::pinocchio
  )
  ament_target_dependencies(test_satellite_collision
    haptic_dmp_learning
    franka_cartesian_control
  )

  ament_add_gtest(test_grasp_launch test/test_grasp_launch.cpp)
  target_link_libraries(test_grasp_launch
    ${PROJECT_NAME}_core
    pinocchio::pinocchio
  )
  ament_target_dependencies(test_grasp_launch
    haptic_dmp_learning
    franka_cartesian_control
  )

  # ---- grasp_planner_node tests (Task 1 of 2) ----
  ament_add_gtest(test_grasp_planner_checks test/test_grasp_planner_checks.cpp)
  target_link_libraries(test_grasp_planner_checks
    grasp_planner_ros_lib
  )

  ament_add_gtest(test_grasp_command_conversion test/test_grasp_command_conversion.cpp)
  target_link_libraries(test_grasp_command_conversion
    grasp_planner_ros_lib
    ${PROJECT_NAME}_core
    pinocchio::pinocchio
  )
  ament_target_dependencies(test_grasp_command_conversion
    rclcpp
    satellite_grasp_msgs
    haptic_dmp_learning
    franka_cartesian_control
  )

  ament_add_gtest(test_theta_convention test/test_theta_convention.cpp)
  target_link_libraries(test_theta_convention
    ${PROJECT_NAME}_core
    pinocchio::pinocchio
  )
  ament_target_dependencies(test_theta_convention
    haptic_dmp_learning
    franka_cartesian_control
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(Eigen3 pinocchio coal haptic_dmp_learning franka_cartesian_control)

ament_package()
