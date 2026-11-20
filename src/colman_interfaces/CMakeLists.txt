cmake_minimum_required(VERSION 3.10)
project(colman_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/WorldStack.msg"
  "msg/WorldState.msg"
  "action/Pick.action"
  "action/Place.action"
  "action/MoveTo.action"
  "action/MovePose.action"
  DEPENDENCIES geometry_msgs builtin_interfaces
)

ament_package()
