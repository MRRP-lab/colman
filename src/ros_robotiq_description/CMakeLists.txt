cmake_minimum_required(VERSION 3.8)
project(ros_robotiq_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Find the build system
find_package(ament_cmake REQUIRED)

# 2. Install the directories
# This command copies your folders to install/share/ros_robotiq_description
install(DIRECTORY
  config
  launch
  rviz
  urdf
  worlds
  DESTINATION share/${PROJECT_NAME}
)

# 3. Finalize the package
ament_package()