cmake_minimum_required(VERSION 3.10)
project(tracking)

# C++ version
set(CMAKE_CXX_STANDARD 17)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Include directories
include_directories(
  include
)

# Build turtlebot3_model node
add_executable(turtlebot3_model
  src/main.cpp
  src/turtlebot3_model.cpp
  src/pid_controller.cpp
)

ament_target_dependencies(turtlebot3_model
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf2_ros
  tf2
)

# Install C++ executable
install(TARGETS
  turtlebot3_model
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files (if you have a headers directory)
install(DIRECTORY include/
  DESTINATION include/
)

# Install Python target_publisher (if you use it)
install(PROGRAMS
  scripts/target_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# Export package
ament_package()
