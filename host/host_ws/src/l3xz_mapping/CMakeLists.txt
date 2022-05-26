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
  Startpoint.msg
  Track.msg
)
add_service_files(
  FILES
  SetWaypoint.srv
  SetStartpoint.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
	CATKIN_DEPENDS nav_msgs std_msgs tf sensor_msgs rospy
)

include_directories(
  $(catkin_INCLUDE_DIRS)
)

catkin_install_python(PROGRAMS
   scripts/transform_odom.py
   scripts/recorder.py
   scripts/dummy_caller.py
   scripts/plotter.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/scripts/geo.py DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/../../devel/lib/${PROJECT_NAME})
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/scripts/logger.py DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/../../devel/lib/${PROJECT_NAME})
