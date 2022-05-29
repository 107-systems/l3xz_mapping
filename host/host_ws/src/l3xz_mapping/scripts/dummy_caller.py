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
from l3xz_mapping.msg import Startpoint
from l3xz_mapping.srv import SetStartpoint, SetStartpointResponse

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

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

def preparedata(x, y):
  data = Odometry()
  data.header.stamp = rospy.Time.now()
  data.header.frame_id = "map"
  data.child_frame_id = "base_link"
  data.pose.pose.position = Vector3(x, y, 0)
  return data

if __name__ == "__main__":
  
#  rospy.wait_for_service('/l3xz/recorder/set_startpoint')
  rospy.init_node("~")
  publisher = rospy.Publisher(rospy.get_param('~odometry_out', '/odom_slam'), Odometry, queue_size = 1)
 
#  try:
#    setter = rospy.ServiceProxy('/l3xz/recorder/set_startpoint', SetStartpoint)
#    p = Startpoint()
#    p.position.x = 0
#    p.position.y = 0
#    p.position.z = 0
#    p.latitude = 49.012240
#    p.longitude = 12.792822
#    resp = setter(p)
#  except rospy.ServiceException as e:
#    print("Service call failed: %s"%e)
  x = 0
  y = 0
  for x in range(0, 25):
    publisher.publish(preparedata(x, y))
    time.sleep(0.5)
  for y in range(0, 50):
    publisher.publish(preparedata(x, y))
    time.sleep(0.5)
  for x in range(25, -50, -1):
    publisher.publish(preparedata(x, y))
    time.sleep(0.5)
  for y in range(50, -50, -1):
    publisher.publish(preparedata(x, y))
    time.sleep(0.5)
  for x in range(-50, 0):
    publisher.publish(preparedata(x, y))
    time.sleep(0.5)
  for y in range(-50, 0):
    publisher.publish(preparedata(x, y))
    time.sleep(0.5)

