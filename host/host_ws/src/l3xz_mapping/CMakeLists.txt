cmake_minimum_required(VERSION 3.0.2)

project(l3xz_mapping)

find_package(catkin REQUIRED COMPONENTS
	nav_msgs
	std_msgs
	tf
	sensor_msgs
	rospy
)

catkin_package(
	CATKIN_DEPENDS nav_msgs std_msgs tf sensor_msgs
)

include_directories(
)

catkin_install_python(PROGRAMS
   scripts/transform_odom.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

