#!/usr/bin/python3
#-*- coding: utf-8 -*-
import os
if 'ROS_NAMESPACE' not in os.environ:
  os.environ['ROS_NAMESPACE'] = 'l3xz'

import rospy
import roslib

import threading

from l3xz_mapping.msg import Waypoint, Track
from l3xz_mapping.srv import SetWaypoint, SetWaypointResponse

mutex = threading.Lock() # Mutex for asynchronous services and publishing.

def set_waypoint_callback(request): # GPIO output service

  global mutex
  mutex.acquire()
  success = True 
  mutex.release()
  return SetWaypointResponse(success)

def main():
  
  rospy.init_node('recorder')

  service_set_waypoint = rospy.Service(rospy.get_name() + '/set_waypoint', SetWaypoint, set_waypoint_callback)
  
  rate = rospy.Rate(100)
  
  global mutex
  while not rospy.is_shutdown():

    mutex.acquire() 
    mutex.release()
    
    rate.sleep()

if __name__ == '__main__':
  main()
