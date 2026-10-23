cmake_minimum_required(VERSION 3.8)
project(satellite_grasp_planner)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "CMAKE_BUILD_TYPE not set: defaulting to Release")
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

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

# ---- Level 1 core library (Eigen3 + Pinocchio + coal, NO ROS) ----
# Depends on haptic_dmp_learning_core (core::ProDMP, for candidate rollouts)
# and franka_cartesian_control_core (core::RobotModel, core::VelocityIkSolver,
# core::computePoseError - reused verbatim for the offline joint-path
# simulation, see core/joint_path_simulator.hpp).
add_library(${PROJECT_NAME}_core
  src/core/approach_clearance.cpp
  src/core/follow_margin.cpp
  src/core/joint_path_simulator.cpp
  src/core/kinematic_feasibility.cpp
  src/core/launch_plan.cpp
  src/core/selection_inputs.cpp
  src/core/start_state.cpp
  src/core/grasp_pose_eval.cpp
  src/core/grasp_s3_selection.cpp
  src/core/grasp_s3_launch.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# coal: signed distances of the approach clearance constraint (core/approach_clearance.cpp), as the planner linked it
# before b3080b5 (find_package(coal REQUIRED), coal::coal).
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

# ---- ROS2 wrapper: one-shot grasp planner node (see docs/DESIGN_NOTES.md) ----
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
)
# grasp_command_conversion.cpp includes grasp_launch_result.hpp, which transitively pulls in
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

# ---- Helpers for the uniform test / analysis targets ----
# add_planner_gtest(<name> [INCLUDE_DIRS ...] [LIBS ...] [DEPS ...])
#   gtest test/<name>.cpp linked to ${PROJECT_NAME}_core + pinocchio::pinocchio, with the
#   haptic_dmp_learning / franka_cartesian_control ament deps; LIBS / DEPS are appended after those,
#   INCLUDE_DIRS (private) are added before them, as in a hand-written target.
#   Only call it inside if(BUILD_TESTING). New test = one line: add_planner_gtest(test_foo)
# add_planner_analysis(<name> [INCLUDE_DIRS ...] [LIBS ...] [DEPS ...])
#   Same for analysis/<name>.cpp (executable), also installed to lib/${PROJECT_NAME}.
#   Only call it inside if(BUILD_ANALYSIS_TOOLS). New tool = one line: add_planner_analysis(foo)
# Anything else (different link order, other sources) stays as explicit lines
# after the call, or the target is written out by hand.
function(add_planner_gtest name)
  cmake_parse_arguments(ARG "" "" "INCLUDE_DIRS;LIBS;DEPS" ${ARGN})
  ament_add_gtest(${name} test/${name}.cpp)
  if(ARG_INCLUDE_DIRS)
    target_include_directories(${name} PRIVATE ${ARG_INCLUDE_DIRS})
  endif()
  target_link_libraries(${name}
    ${PROJECT_NAME}_core
    pinocchio::pinocchio
    ${ARG_LIBS}
  )
  ament_target_dependencies(${name}
    haptic_dmp_learning
    franka_cartesian_control
    ${ARG_DEPS}
  )
endfunction()

function(add_planner_analysis name)
  cmake_parse_arguments(ARG "" "" "INCLUDE_DIRS;LIBS;DEPS" ${ARGN})
  add_executable(${name} analysis/${name}.cpp)
  if(ARG_INCLUDE_DIRS)
    target_include_directories(${name} PRIVATE ${ARG_INCLUDE_DIRS})
  endif()
  target_link_libraries(${name}
    ${PROJECT_NAME}_core
    pinocchio::pinocchio
    ${ARG_LIBS}
  )
  ament_target_dependencies(${name}
    haptic_dmp_learning
    franka_cartesian_control
    ${ARG_DEPS}
  )
  install(TARGETS ${name}
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
endfunction()

# ---- Optional offline analysis executables (default OFF: no existing build changes) ----
option(BUILD_ANALYSIS_TOOLS "Build offline analysis executables" OFF)
if(BUILD_ANALYSIS_TOOLS)
  add_planner_analysis(orientation_law_compare)
  add_planner_analysis(orientation_path_check)
  add_planner_analysis(s3_landscape INCLUDE_DIRS analysis)
  add_planner_analysis(s3_clearance)
  add_planner_analysis(s3_followmargin_check)
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  add_planner_gtest(test_manipulability)
  add_planner_gtest(test_joint_path_simulator)
  add_planner_gtest(test_kinematic_feasibility)
  add_planner_gtest(test_follow_margin)
  add_planner_gtest(test_selection_inputs)
  add_planner_gtest(test_start_state)
  add_planner_gtest(test_launch_delay)
  add_planner_gtest(test_launch_plan)

  # ---- grasp_planner_node tests ----
  ament_add_gtest(test_grasp_planner_checks test/test_grasp_planner_checks.cpp)
  target_link_libraries(test_grasp_planner_checks
    grasp_planner_ros_lib
  )

  # not uniform: grasp_planner_ros_lib is linked BEFORE ${PROJECT_NAME}_core and rclcpp/satellite_grasp_msgs
  # come first in the ament deps, so the helper (core first, fixed deps first) would change the order.
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

  add_planner_gtest(test_grasp_s3_selection)
  add_planner_gtest(test_theta_convention)
  add_planner_gtest(test_grasp_pose_eval)
  add_planner_gtest(test_approach_clearance)
  add_planner_gtest(test_s3_landscape_log INCLUDE_DIRS analysis)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(Eigen3 pinocchio coal haptic_dmp_learning franka_cartesian_control)

ament_package()
