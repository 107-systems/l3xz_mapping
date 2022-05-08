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
logger = None

def set_waypoint_callback(request):
  global startposition
  global startpoint
  global logger
  global mutex
  mutex.acquire()
  success = True 
  if startpoint is not None and startposition is not None:
    logpoint = startpoint.point_from_delta(request.waypoint.position.x - startposition.x, request.waypoint.position.y - startposition.y)
    logger.log(request.waypoint.header.stamp.secs, request.waypoint.header.stamp.nsecs * 1000, logpoint.utm_zone, logpoint.utm_easting, logpoint.utm_northing) 
  mutex.release()
  return SetWaypointResponse(success)

def set_startpoint_callback(request):
  global startposition
  global startpoint
  global mutex
  mutex.acquire()
  success = True 
  startposition = request.startpoint.position
  startpoint = GeoPoint(request.startpoint.latitude, request.startpoint.longitude)
  print(request)
  mutex.release()
  return SetStartpointResponse(success)

def main():
  global logger

  rospy.init_node('recorder')

  service_set_waypoint = rospy.Service(rospy.get_name() + '/set_waypoint', SetWaypoint, set_waypoint_callback)
  service_set_startpoint = rospy.Service(rospy.get_name() + '/set_startpoint', SetStartpoint, set_startpoint_callback)
 
  logfile = rospy.get_param("~logfile", "/home/l3xz/log.txt")
  logger = Logger(logfile)

  rate = rospy.Rate(10)
  
  global mutex
  while not rospy.is_shutdown():

    mutex.acquire() 
    mutex.release()
    
    rate.sleep()

if __name__ == '__main__':
  main()
