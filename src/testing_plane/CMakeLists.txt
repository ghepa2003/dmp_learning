cmake_minimum_required(VERSION 3.8)
project(testing_plane)

find_package(ament_cmake REQUIRED)

# Install directories
install(
    DIRECTORY launch xacro config scripts
    DESTINATION share/${PROJECT_NAME}
)

ament_package()

