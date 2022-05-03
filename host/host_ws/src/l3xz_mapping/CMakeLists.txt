cmake_minimum_required(VERSION 3.0.2)

project(l3xz_mapping)

find_package(catkin REQUIRED COMPONENTS
	nav_msgs
	std_msgs
	tf
	sensor_msgs
  geometry_msgs
	rospy
  message_generation
)

add_message_files(
  FILES
  Waypoint.msg
  Track.msg
)
add_service_files(
  FILES
  SetWaypoint.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
	CATKIN_DEPENDS nav_msgs std_msgs tf sensor_msgs
)

include_directories(
)

catkin_install_python(PROGRAMS
   scripts/transform_odom.py
   scripts/recorder.py
   scripts/dummy_caller.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

