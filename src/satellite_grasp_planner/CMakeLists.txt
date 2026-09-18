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
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_core
  Eigen3::Eigen
  pinocchio::pinocchio
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
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(Eigen3 pinocchio haptic_dmp_learning franka_cartesian_control)

ament_package()
