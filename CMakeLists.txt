cmake_minimum_required(VERSION 3.5)
project(rs)

# Required for Python launch files
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Include headers
include_directories(
  include
)

# Add executable for IMU subscriber node
add_executable(RSNode src/RSNode.cpp)
ament_target_dependencies(RSNode rclcpp sensor_msgs tf2_ros)

# Install launch and config directories
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# Install header files
install(
  DIRECTORY include/
  DESTINATION include
)

# Install IMU subscriber node
install(TARGETS
  RSNode
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
