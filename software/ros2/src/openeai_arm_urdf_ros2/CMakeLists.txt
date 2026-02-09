cmake_minimum_required(VERSION 3.12)

project(openeai_arm_urdf_ros2)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf meshes config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
