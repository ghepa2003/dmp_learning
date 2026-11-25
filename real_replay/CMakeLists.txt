cmake_minimum_required(VERSION 3.16)
project(real_replay LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

# ---- paths -----------------------------------------------------------------------------------
# Thesis sources (existing packages are compiled from source, unmodified; no ament/colcon here).
set(THESIS_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../src" CACHE PATH "thesis_ws/src")
set(FCC_DIR "${THESIS_SRC_DIR}/franka_cartesian_control")

# The libfranka NON-RT build tree (NOT installed system-wide). The system libfranka 0.9.3 (RT
# variant) must never be picked up.
set(FRANKA_NONRT_BUILD_DIR
    "/root/comanipulation_dualpanda/libfranka-active-control-non-rt/build"
    CACHE PATH "Build directory of libfranka-active-control-non-rt")

if(NOT EXISTS "${FRANKA_NONRT_BUILD_DIR}/FrankaConfig.cmake")
  message(FATAL_ERROR
    "FrankaConfig.cmake not found in FRANKA_NONRT_BUILD_DIR='${FRANKA_NONRT_BUILD_DIR}'.\n"
    "Build libfranka-active-control-non-rt first, or pass -DFRANKA_NONRT_BUILD_DIR=<its build dir>.")
endif()

# Pinocchio comes from ros-humble-pinocchio (/opt/ros/humble) in the container.
if(EXISTS "/opt/ros/humble")
  list(APPEND CMAKE_PREFIX_PATH "/opt/ros/humble")
endif()

# ---- dependencies ----------------------------------------------------------------------------
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)

# Config-mode lookup restricted to the non-rt build directory: no default/system paths.
find_package(Franka REQUIRED CONFIG PATHS "${FRANKA_NONRT_BUILD_DIR}" NO_DEFAULT_PATH)

get_filename_component(_franka_dir_real "${Franka_DIR}" REALPATH)
get_filename_component(_franka_want_real "${FRANKA_NONRT_BUILD_DIR}" REALPATH)
if(NOT _franka_dir_real STREQUAL _franka_want_real)
  message(FATAL_ERROR "Franka_DIR='${Franka_DIR}' does not point to '${FRANKA_NONRT_BUILD_DIR}'. "
                      "Remove the stale cache entry (rm -rf the build dir) and reconfigure.")
endif()

get_target_property(_franka_so Franka::Franka LOCATION)
if(NOT _franka_so)
  get_target_property(_franka_so Franka::Franka IMPORTED_LOCATION)
endif()
if(_franka_so)
  get_filename_component(_franka_so_real "${_franka_so}" REALPATH)
  string(FIND "${_franka_so_real}" "${_franka_want_real}/" _franka_pos)
  if(NOT _franka_pos EQUAL 0)
    message(FATAL_ERROR "Franka::Franka resolves to '${_franka_so_real}', which is NOT under '${_franka_want_real}'.")
  endif()
  message(STATUS "libfranka (non-rt): ${_franka_so_real}")
else()
  message(WARNING "Could not determine the Franka::Franka library location to double-check it.")
endif()

if(TARGET yaml-cpp::yaml-cpp)
  set(YAML_TARGET yaml-cpp::yaml-cpp)
else()
  set(YAML_TARGET yaml-cpp)
endif()

# ---- existing thesis code, compiled from source and unmodified ----------------------------------
set(HDL_DIR "${THESIS_SRC_DIR}/haptic_dmp_learning")
set(SGP_DIR "${THESIS_SRC_DIR}/satellite_grasp_planner")

# franka_cartesian_control core (+ the DLS velocity IK used by the planner's JointPathSimulator)
add_library(fcc_core STATIC
  "${FCC_DIR}/src/core/robot_model.cpp"
  "${FCC_DIR}/src/core/impedance_solver.cpp"
  "${FCC_DIR}/src/core/velocity_ik_solver.cpp"
)
target_include_directories(fcc_core PUBLIC "${FCC_DIR}/include")
target_link_libraries(fcc_core PUBLIC Eigen3::Eigen pinocchio::pinocchio)

# haptic_dmp_learning core: only what ProDMP / QuaternionDMP loading needs (no ROS, no messages).
# If the link ever reports an undefined symbol, add the remaining files of core/ (11 in total, see
# haptic_dmp_learning/CMakeLists.txt:34-46).
add_library(hdl_core STATIC
  "${HDL_DIR}/src/core/prodmp.cpp"
  "${HDL_DIR}/src/core/prodmp_io.cpp"
  "${HDL_DIR}/src/core/quaternion_dmp.cpp"
  "${HDL_DIR}/src/core/dmp_io.cpp"
  "${HDL_DIR}/src/core/dmp.cpp"
)
target_include_directories(hdl_core PUBLIC "${HDL_DIR}/include")
target_link_libraries(hdl_core PUBLIC Eigen3::Eigen ${YAML_TARGET})

# satellite_grasp_planner: ONLY the ROS-free JointPathSimulator (manipulability.hpp is header-only).
add_library(sgp_jps STATIC "${SGP_DIR}/src/core/joint_path_simulator.cpp")
target_include_directories(sgp_jps PUBLIC "${SGP_DIR}/include")
target_link_libraries(sgp_jps PUBLIC fcc_core Eigen3::Eigen)

# ---- real_replay -----------------------------------------------------------------------------
add_library(rr_lib STATIC
  src/config.cpp
  src/controller_core.cpp
  src/safety.cpp
  src/logger.cpp
  src/target_source.cpp
  src/sha256.cpp
  src/prodmp_source.cpp
  src/path_check.cpp
  src/preflight_checks.cpp
)
target_include_directories(rr_lib PUBLIC include)
target_link_libraries(rr_lib PUBLIC fcc_core hdl_core sgp_jps ${YAML_TARGET} Eigen3::Eigen)
target_compile_options(rr_lib PRIVATE -Wall -Wextra)

# stage 2 task layer (pure logic + runtime threads; no libfranka, no ROS)
add_library(task_lib STATIC src/task_logic.cpp src/task_runtime.cpp)
target_include_directories(task_lib PUBLIC include)
target_link_libraries(task_lib PUBLIC rr_lib ${YAML_TARGET} Eigen3::Eigen Threads::Threads)
target_compile_options(task_lib PRIVATE -Wall -Wextra)

add_executable(real_replay src/main.cpp)
target_link_libraries(real_replay PRIVATE task_lib rr_lib Franka::Franka Threads::Threads)
target_compile_options(real_replay PRIVATE -Wall -Wextra)

# The non-rt and the system libfranka share name and SOVERSION (both 0.9.3): force the build
# directory first in the runtime search path. DT_RPATH (--disable-new-dtags) beats LD_LIBRARY_PATH
# and the ld.so cache, DT_RUNPATH does not.
set_target_properties(real_replay PROPERTIES
  BUILD_WITH_INSTALL_RPATH OFF
  BUILD_RPATH "${FRANKA_NONRT_BUILD_DIR}")
target_link_options(real_replay PRIVATE -Wl,--disable-new-dtags)

# ---- tests (no libfranka needed; run with ctest or directly) ---------------------------------
option(REAL_REPLAY_BUILD_TESTS "Build the real_replay tests" ON)
if(REAL_REPLAY_BUILD_TESTS)
  enable_testing()
  set(RR_CONFIG "${CMAKE_CURRENT_SOURCE_DIR}/config/real_replay.yaml")
  set(RR_URDF "${CMAKE_CURRENT_SOURCE_DIR}/urdf/fer_nohand.urdf")

  add_executable(test_sources test/test_sources.cpp)
  target_link_libraries(test_sources PRIVATE rr_lib)
  add_test(NAME sources COMMAND test_sources "${RR_CONFIG}")

  add_executable(test_damping_design test/test_damping_design.cpp)
  target_link_libraries(test_damping_design PRIVATE rr_lib)
  add_test(NAME damping_design COMMAND test_damping_design "${RR_CONFIG}")

  add_executable(test_damping_torque test/test_damping_torque.cpp)
  target_link_libraries(test_damping_torque PRIVATE rr_lib)
  add_test(NAME damping_torque COMMAND test_damping_torque "${RR_CONFIG}" "${RR_URDF}")
  set_tests_properties(damping_torque PROPERTIES SKIP_RETURN_CODE 77)

  add_executable(test_preflight_checks test/test_preflight_checks.cpp)
  target_link_libraries(test_preflight_checks PRIVATE rr_lib)
  add_test(NAME preflight_checks COMMAND test_preflight_checks "${RR_CONFIG}" "${RR_URDF}")
  set_tests_properties(preflight_checks PROPERTIES SKIP_RETURN_CODE 77)

  add_executable(test_start_pose test/test_start_pose.cpp)
  target_link_libraries(test_start_pose PRIVATE rr_lib)
  add_test(NAME start_pose COMMAND test_start_pose "${CMAKE_CURRENT_SOURCE_DIR}/config/replay_demo1014_f030_hand_kr30_ffa100.yaml" "${RR_CONFIG}")

  add_executable(test_tracking_ignore test/test_tracking_ignore.cpp)
  target_link_libraries(test_tracking_ignore PRIVATE rr_lib)
  add_test(NAME tracking_ignore COMMAND test_tracking_ignore "${RR_CONFIG}")

  add_executable(test_friction_ff test/test_friction_ff.cpp)
  target_link_libraries(test_friction_ff PRIVATE rr_lib)
  add_test(NAME friction_ff COMMAND test_friction_ff "${RR_CONFIG}" "${RR_URDF}")
  set_tests_properties(friction_ff PROPERTIES SKIP_RETURN_CODE 77)

  add_executable(test_replay_dryrun test/test_replay_dryrun.cpp)
  target_link_libraries(test_replay_dryrun PRIVATE rr_lib)
  # Count heap allocations: operator new is replaced in the test; direct malloc-family calls
  # (Eigen, Pinocchio code compiled into this binary) are intercepted with the linker's --wrap.
  target_link_options(test_replay_dryrun PRIVATE
    -Wl,--wrap=malloc -Wl,--wrap=calloc -Wl,--wrap=realloc -Wl,--wrap=posix_memalign -Wl,--wrap=aligned_alloc)
  add_test(NAME replay_dryrun COMMAND test_replay_dryrun "${RR_CONFIG}" "${RR_URDF}")
  set_tests_properties(replay_dryrun PROPERTIES SKIP_RETURN_CODE 77)

  add_executable(test_j7_profile test/test_j7_profile.cpp)
  target_link_libraries(test_j7_profile PRIVATE rr_lib)
  add_test(NAME j7_profile COMMAND test_j7_profile)
endif()

# ---- satellite J7 driver (standalone tool: joint PD, only J7 moves; no ReplayLoop) -----------------
add_executable(satellite_j7 tools/satellite_j7.cpp)
target_link_libraries(satellite_j7 PRIVATE rr_lib Franka::Franka Threads::Threads)
target_compile_options(satellite_j7 PRIVATE -Wall -Wextra)
set_target_properties(satellite_j7 PROPERTIES
  BUILD_WITH_INSTALL_RPATH OFF
  BUILD_RPATH "${FRANKA_NONRT_BUILD_DIR}")
target_link_options(satellite_j7 PRIVATE -Wl,--disable-new-dtags)

# ---- hw stage 1: robot 2 "satellite" process (hw/hw_config.yaml, UDP state/commands) ----------------
# Pure helpers (no libfranka): hw_config loader; satellite_phase.hpp / satellite_udp.hpp are header-only.
add_library(sat_lib STATIC src/hw_config.cpp)
target_include_directories(sat_lib PUBLIC include)
target_link_libraries(sat_lib PUBLIC Eigen3::Eigen ${YAML_TARGET})
target_compile_options(sat_lib PRIVATE -Wall -Wextra)

add_executable(satellite_node tools/satellite_node.cpp)
target_link_libraries(satellite_node PRIVATE sat_lib rr_lib Franka::Franka Threads::Threads)
target_compile_options(satellite_node PRIVATE -Wall -Wextra)
set_target_properties(satellite_node PROPERTIES
  BUILD_WITH_INSTALL_RPATH OFF
  BUILD_RPATH "${FRANKA_NONRT_BUILD_DIR}")
target_link_options(satellite_node PRIVATE -Wl,--disable-new-dtags)

if(REAL_REPLAY_BUILD_TESTS)
  set(HW_CONFIG "${CMAKE_CURRENT_SOURCE_DIR}/hw/hw_config.yaml")

  add_executable(test_satellite_phase test/test_satellite_phase.cpp)
  target_link_libraries(test_satellite_phase PRIVATE sat_lib)
  add_test(NAME satellite_phase COMMAND test_satellite_phase "${HW_CONFIG}")

  add_executable(test_satellite_udp test/test_satellite_udp.cpp)
  target_link_libraries(test_satellite_udp PRIVATE sat_lib Threads::Threads)
  add_test(NAME satellite_udp COMMAND test_satellite_udp)

  find_program(PYTHON3_EXE python3)
  if(PYTHON3_EXE)
    add_test(NAME hw_plan COMMAND ${PYTHON3_EXE} "${CMAKE_CURRENT_SOURCE_DIR}/hw/test/test_hw_plan.py")
    add_test(NAME reach_check COMMAND ${PYTHON3_EXE} "${CMAKE_CURRENT_SOURCE_DIR}/hw/test/test_reach_check.py")
    # dry-run end-to-end (about 25 s): satellite_node --dry-run --autostart + UDP listener (omega within 1 %)
    add_test(NAME satellite_dryrun_e2e COMMAND ${PYTHON3_EXE} "${CMAKE_CURRENT_SOURCE_DIR}/hw/test/test_satellite_e2e.py"
             $<TARGET_FILE:satellite_node> "${HW_CONFIG}" "${CMAKE_CURRENT_BINARY_DIR}/logs")
    set_tests_properties(satellite_dryrun_e2e PROPERTIES TIMEOUT 120)
  endif()
endif()

# ---- stadio 2: ProDmpSource goal mode equivalence ----------------------------------------------------
if(REAL_REPLAY_BUILD_TESTS)
  add_executable(test_prodmp_goal test/test_prodmp_goal.cpp)
  target_link_libraries(test_prodmp_goal PRIVATE rr_lib)
  add_test(NAME prodmp_goal COMMAND test_prodmp_goal "${CMAKE_CURRENT_SOURCE_DIR}/../runs/20261014T150851_prodmp/weights.yaml")
  add_executable(test_slerp_hw_goal test/test_slerp_hw_goal.cpp)
  target_link_libraries(test_slerp_hw_goal PRIVATE rr_lib)
  add_test(NAME slerp_hw_goal COMMAND test_slerp_hw_goal "${CMAKE_CURRENT_SOURCE_DIR}/../runs/20261124T165020_prodmp/weights.yaml")
  add_executable(test_task_logic test/test_task_logic.cpp)
  target_link_libraries(test_task_logic PRIVATE task_lib)
  add_test(NAME task_logic COMMAND test_task_logic "${CMAKE_CURRENT_SOURCE_DIR}/hw/hw_config.yaml"
           "${CMAKE_CURRENT_SOURCE_DIR}/hw/examples/grasp_command_nominale.yaml")
  find_program(PYTHON3_EXE_TASK python3)
  if(PYTHON3_EXE_TASK)
    # hw.py run --dry-run end-to-end (about 2.5 min): omega -2, omega -3, infeasible x2, satellite killed
    add_test(NAME task_dryrun_e2e COMMAND ${PYTHON3_EXE_TASK} "${CMAKE_CURRENT_SOURCE_DIR}/hw/test/test_task_e2e.py"
             "${CMAKE_CURRENT_BINARY_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/hw/hw_config.yaml"
             "${CMAKE_CURRENT_SOURCE_DIR}/../runs/20261014T150851_prodmp/weights.yaml")
    set_tests_properties(task_dryrun_e2e PROPERTIES TIMEOUT 400)
  endif()
endif()
