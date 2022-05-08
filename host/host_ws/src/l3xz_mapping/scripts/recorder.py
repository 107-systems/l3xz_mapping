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

mutex = threading.Lock() # Mutex for asynchronous services and publishing.

startposition = None
startpoint = None

def set_waypoint_callback(request):
  global startposition
  global startpoint
  global mutex
  mutex.acquire()
  success = True 
  if startpoint is not None and startposition is not None:
    print(startpoint.point_from_delta(request.waypoint.position.x - startposition.x, request.waypoint.position.y - startposition.y))
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
  rospy.init_node('recorder')

  service_set_waypoint = rospy.Service(rospy.get_name() + '/set_waypoint', SetWaypoint, set_waypoint_callback)
  service_set_startpoint = rospy.Service(rospy.get_name() + '/set_startpoint', SetStartpoint, set_startpoint_callback)
  
  rate = rospy.Rate(100)
  
  global mutex
  while not rospy.is_shutdown():

    mutex.acquire() 
    mutex.release()
    
    rate.sleep()

if __name__ == '__main__':
  main()
