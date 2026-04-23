cmake_minimum_required(VERSION 3.8)
project(ekf_launch)

find_package(ament_cmake REQUIRED)

# Install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config/param files if present
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  OPTIONAL   # won't fail if the directory doesn't exist yet
)

ament_package()