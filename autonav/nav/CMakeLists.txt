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

# robot_side: safety gate (only publisher of /cmd_vel_safe), Nav2 restart helper, and its approved config
install(
  PROGRAMS ../../robot_side/safety/safety_gate.py ../../robot_side/safety/safety_core.py
           ../../robot_side/tools/nav2_manager.py ../../robot_side/tools/limits_check.py ../../robot_side/tools/nav2_proc.py
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY ../../robot_side/config/
  DESTINATION share/${PROJECT_NAME}/autotune_config
  FILES_MATCHING PATTERN "safety_config.*" PATTERN "zone.npz" PATTERN "commissioned.json"
)

# Install directories
install(
  DIRECTORY launch params urdf config maps
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

