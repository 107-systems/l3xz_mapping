#!/usr/bin/env python3
#-*- coding: utf-8 -*-
import roslib
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point

odom_msg = None

def callback_odom(self, msg):
   odom_msg = msg 

if __name__ == '__main__':
    rospy.init_node('odom_Waypoint*')

    odomsub = rospy.Subscriber(rospy.get_param('~odom_topic', '/odoms_slam'), Odometry, queue_size = 1, callback_odom)
    global servie_topic
    service_topic = rospy.get_param('~service_topic',  '/l3xz/recorder/set_waypoint')
    setter = rospy.ServiceProxy(service_topic, SetWaypoint)
    
    tag = rospy.get_param('~tag',  'robot')
    
    rate = rospy.Rate(rospy.get_param('~rate_hz', 1))
     
    while not rospy.is_shutdown():
        p = Waypoint()
        p.position = msg.pose.pose.position
        p.tag = tag
        resp = setter(p)
      except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

        rate.sleep()
