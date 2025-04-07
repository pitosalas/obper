cmake_minimum_required(VERSION 3.8)
project(obstacle_perception)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObstacleStatus.msg"
  DEPENDENCIES
)

# Executables
add_executable(my_costmap_node src/my_costmap_node.cpp)
ament_target_dependencies(my_costmap_node
  rclcpp
  rclcpp_lifecycle
  nav2_costmap_2d
  tf2_ros
)

add_executable(standalone_costmap src/standalone_costmap.cpp)
ament_target_dependencies(standalone_costmap
  rclcpp
  rclcpp_lifecycle
  nav2_costmap_2d
  tf2_ros
)

# Install C++ nodes
install(TARGETS
  my_costmap_node
  standalone_costmap
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()