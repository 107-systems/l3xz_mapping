#!/usr/bin/env python 
#-*- coding: utf-8 -*-
## @package transform_odom
#  Creates odometry message from timestamped tf parent child relation
import roslib
import rospy
import math
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

if __name__ == '__main__':
    rospy.init_node('map_odom')

    listener = tf.TransformListener()
    publisher = rospy.Publisher(rospy.get_param('~odometry_out', '/odom_slam'), Odometry, queue_size = 1)
    parent = rospy.get_param('~parent_frame', '/map')
    child = rospy.get_param('~child_frame', '/base_link')
    
    lastPose = None
    lastQ = None
    pose = None
    q = None

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            lastPose = pose
            lastQ = q
            (pose ,q) = listener.lookupTransform(parent, child, rospy.Time(0))
            data = Odometry()
            data.header.stamp = rospy.Time.now()
            data.header.frame_id = parent
            data.child_frame_id = child
            data.pose.pose.position = Vector3(pose[0], pose[1], pose[2])
            data.pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

            if lastPose is not None and lastQ is not None:
              data.twist.twist.linear = Vector3(pose[0] - lastPose[0], pose[1] - lastPose[1], pose[2] - lastPose[2])
              euler = tf.transformations.euler_from_quaternion(q)
              lastEuler = tf.transformations.euler_from_quaternion(lastQ)
              data.twist.twist.angular = Vector3(euler[0] - lastEuler[0], euler[1] - lastEuler[1], euler[2] - lastEuler[2])
            publisher.publish(data)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        rate.sleep()
