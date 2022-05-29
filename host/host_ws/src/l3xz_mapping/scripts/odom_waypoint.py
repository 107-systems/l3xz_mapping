#!/usr/bin/env python3
#-*- coding: utf-8 -*-
import roslib
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from l3xz_mapping.msg import Waypoint
from l3xz_mapping.srv import SetWaypoint, SetWaypointResponse


def callback_odom(msg):
   global odom_msg
   odom_msg = msg 

if __name__ == '__main__':
    global odom_msg
    odom_msg = None
    global servie_topic
    rospy.init_node('odom_waypoint')

    odomsub = rospy.Subscriber(rospy.get_param('~odom_topic', '/odom_slam'), Odometry, callback_odom)
    service_topic = rospy.get_param('~service_topic',  '/l3xz/odom_recorder/set_waypoint')
    setter = rospy.ServiceProxy(service_topic, SetWaypoint)
    
    tag = rospy.get_param('~tag',  'robot')
    
    rate = rospy.Rate(rospy.get_param('~rate_hz', 1))
     
    while not rospy.is_shutdown():
      if odom_msg is not None:
        try:
          p = Waypoint()
          p.header.stamp = rospy.Time.now()
          p.position = odom_msg.pose.pose.position
          p.tag = tag
          resp = setter(p)
        except rospy.ServiceException as e:
          print("Service call failed: %s"%e)

      rate.sleep()
