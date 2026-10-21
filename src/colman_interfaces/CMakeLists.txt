cmake_minimum_required(VERSION 3.10)
project(colman_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Pick.action"
  "action/Place.action"
  "action/MoveTo.action"
  DEPENDENCIES geometry_msgs
)

ament_package()
