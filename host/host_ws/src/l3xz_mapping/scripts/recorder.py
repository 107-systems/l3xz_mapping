#!/usr/bin/python3
#-*- coding: utf-8 -*-
import os
if 'ROS_NAMESPACE' not in os.environ:
  os.environ['ROS_NAMESPACE'] = 'l3xz'

import rospy
import roslib

import threading

from l3xz_mapping.msg import Startpoint, Waypoint, Track
from l3xz_mapping.srv import SetWaypoint, SetWaypointResponse, SetStartpoint, SetStartpointResponse

from geo import GeoPoint
from logger import Logger

mutex = threading.Lock()

startposition = None
startpoint = None
bearing = 0
logger = None
track = []

def set_waypoint_callback(request):
  global startposition
  global startpoint
  global logger
  global track
  global bearing
  global mutex
  mutex.acquire()
  success = True 
  if startpoint is not None and startposition is not None:
    track.append(request.waypoint)
    logpoint = startpoint.point_from_delta(request.waypoint.position.x - startposition.x, request.waypoint.position.y - startposition.y, bearing)
    logger.log(request.waypoint.header.stamp.secs, request.waypoint.header.stamp.nsecs * 1000, logpoint) 
  mutex.release()
  return SetWaypointResponse(success)

def set_startpoint_callback(request): 
  global startposition 
  global startpoint
  global bearing
  global mutex
  mutex.acquire()
  success = True 
  startposition = request.startpoint.position
  startpoint = GeoPoint(request.startpoint.latitude, request.startpoint.longitude)
  bearing = request.startpoint.bearing 
  rospy.loginfo(request)
  mutex.release()
  return SetStartpointResponse(success)

def main():
  global logger
  global mutex
  global track

  rospy.init_node('recorder')

  service_set_waypoint = rospy.Service(rospy.get_name() + '/set_waypoint', SetWaypoint, set_waypoint_callback)
  service_set_startpoint = rospy.Service(rospy.get_name() + '/set_startpoint', SetStartpoint, set_startpoint_callback)
 
  logpath = rospy.get_param("~logpath", "/home/l3xz/log.txt")
  artifact_topics = rospy.get_param("~artifacts", []) 
  
  logger = Logger(logpath, artifact_topics)

  rate = rospy.Rate(rospy.get_param("~track_publishing_rate", 1))
  track_publisher = None
  if rospy.get_param("~publish_track", True):
    track_publisher = rospy.Publisher(rospy.get_name() + "/track",
          Track, queue_size = 1)
  
  while not rospy.is_shutdown():

    mutex.acquire() 
    if track_publisher is not None and len(track) > 0:
      msg = Track()
      msg.description = rospy.get_name()
      msg.track = track
      track_publisher.publish(msg)
    mutex.release()    
    rate.sleep()

if __name__ == '__main__':
  main()
