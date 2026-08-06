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
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME}_core
  src/core/demonstration_recorder.cpp
  src/core/dmp.cpp
  src/core/quaternion_dmp.cpp  
  src/core/dmp_io.cpp
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
)

install(TARGETS
  ${PROJECT_NAME}_core
  haptic_dmp_wrapper_node
  dmp_gazebo_executor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
