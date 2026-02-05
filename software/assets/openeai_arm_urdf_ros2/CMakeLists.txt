cmake_minimum_required(VERSION 3.12)

project(OpenEAIArmarm_urdf_ros2)

# find_package(catkin REQUIRED)

# catkin_package()

# find_package(roslaunch)

# foreach(dir config launch meshes urdf)
# 	install(DIRECTORY ${dir}/
# 		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
# endforeach(dir)

find_package(ament_cmake REQUIRED)
# find_package(urdf REQUIRED)
# find_package(robot_state_publisher REQUIRED)
# find_package(rviz2 REQUIRED)

install(
  DIRECTORY urdf meshes config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
