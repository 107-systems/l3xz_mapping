#!/usr/bin/python3
#-*- coding: utf-8 -*-
from __future__ import print_function
import os
if 'ROS_NAMESPACE' not in os.environ:
  os.environ['ROS_NAMESPACE'] = 'l3xz'

import sys

import rospy
import roslib
import time

from l3xz_mapping.msg import Waypoint
from l3xz_mapping.srv import SetWaypoint, SetWaypointResponse

def set_waypoint_client(x, y, tag):
  rospy.wait_for_service('/l3xz/recorder/set_waypoint')
  try:
    setter = rospy.ServiceProxy('/l3xz/recorder/set_waypoint', SetWaypoint)
    p = Waypoint()
    p.position.x = x
    p.position.y = y
    p.position.z = 0
    p.tag = tag
    resp = setter(p)
    print(str(x) + ', ' + str(y) + ', ' + tag + ', ' + str(resp))
  except rospy.ServiceException as e:
    print("Service call failed: %s"%e)

if __name__ == "__main__":
  for x in range(0, 50):
    for y in range(0, 50):
      set_waypoint_client(x, y, "test") 
      time.sleep(0.1)
