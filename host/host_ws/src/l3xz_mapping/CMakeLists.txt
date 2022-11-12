cmake_minimum_required(VERSION 3.0.2)

project(l3xz_mapping)

set(SOURCELIST
  src/${PROJECT_NAME}/timer.cpp
  src/${PROJECT_NAME}/pose.cpp
  src/${PROJECT_NAME}/map.cpp
  src/${PROJECT_NAME}/model/base/model_base.cpp
  src/${PROJECT_NAME}/model/model_lidar.cpp
)

add_definitions(-std=c++17 -Wall -g)

find_package(catkin REQUIRED COMPONENTS
	nav_msgs
	std_msgs
	tf
	sensor_msgs
    geometry_msgs
	rospy
    message_generation
    image_transport
    tflistener
)

add_message_files(
  FILES
  Waypoint.msg
  Startpoint.msg
  Track.msg
)

find_package(PCL 1.8 REQUIRED)
find_package(OpenCV 4.5.1 REQUIRED COMPONENTS)

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
  include
  ${catkin_INCLUDE_DIRS}
  SYSTEM ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  ${SOURCELIST}
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(knownposition_node src/${PROJECT_NAME}/knownposition_node.cpp ${SOURCELIST})

set_target_properties(knownposition_node PROPERTIES OUTPUT_NAME knownposition_node PREFIX "")

add_dependencies(knownposition_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(knownposition_node
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
 )

install(TARGETS knownposition_node
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
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
