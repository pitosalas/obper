find_package(REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObstacleStatus.msg"
  DEPENDENCIES
)

