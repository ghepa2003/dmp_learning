cmake_minimum_required(VERSION 3.8)
project(free_target_object)

find_package(ament_cmake REQUIRED)

# Pure data package: model description + launch. Nothing to compile.
install(
  DIRECTORY launch models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
