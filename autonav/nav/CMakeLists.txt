cmake_minimum_required(VERSION 3.5)
project(slamurai_nav)

# Required for Python launch files
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python executable (main.py in src)
install(
  PROGRAMS src/main.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(
  DIRECTORY launch params urdf config maps
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

