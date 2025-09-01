cmake_minimum_required(VERSION 3.5)
project(imu_madgwick_dashing)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(madgwick_node src/madgwick_node.cpp)

ament_target_dependencies(
  madgwick_node
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
)

install(TARGETS
  madgwick_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

