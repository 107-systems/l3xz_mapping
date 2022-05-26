#!/usr/bin/python3
#-*- coding: utf-8 -*-
import os
if 'ROS_NAMESPACE' not in os.environ:
  os.environ['ROS_NAMESPACE'] = 'l3xz'

import rospy
import roslib

import math
import threading
import cv2

from l3xz_mapping.msg import Waypoint, Track

class Plotter:

  def __init__(self, src_img, dest_img, resolution, bearing_offset, start, track_topic, opi_topics = []):
    self._img = cv2.imread(src_img)
    self._height = self._img.shape[0]
    self._dest_img_path = dest_img
    self._resolution = resolution
    self._bearing = bearing_offset
    if 0 > self._bearing:
      self._bearing += 360
    self._start = start

    self._opi_subscribers = []
    for opi in opi_topics:
      self._opi_subscribers.append(rospy.Subscriber(opi, Track, self._update_opi, queue_size = 1))

    self._opis = []
    self._nr_opis = len(opi_topics)
    self._track = None
    self._track_subscriber = rospy.Subscriber(track_topic, Track, self._update_track, queue_size = 1)

  def _update_opi(self, opi):
      for o in self._opis:
        if o.description == opi.description:
          o = opi
          return
      self._opis.append(opi)
          
  def _update_track(self, track):
    self._track = track

  def _position(self, x, y):
    d = math.sqrt(x ** 2 + y ** 2)
    bearing = self._bearing + math.atan2(y, x) / math.pi * 180
    return (int(self._start[0] + math.sin(bearing / 180.0 * math.pi) * d), self._height - int(self._start[1] + math.cos(bearing / 180.0 * math.pi) * d))

  def plot(self):
    if self._track is not None:
      trackpoints = self._track.track
      if len(trackpoints) > 1:
        last = self._position(trackpoints[0].position.x, trackpoints[0].position.y)

        for i in range(1, len(trackpoints)):
          current = self._position(trackpoints[i].position.x, trackpoints[i].position.y)
          try:
            self._img = cv2.line(self._img, last, current, (3, 248, 252), 1)
          except:
            pass
          last = current
    for opi in self._opis:
      trackpoints = opi.track
      for t in trackpoints:
        pos = self._position(t.x, t.y)
        self._img = cv2.line(self._img, (pos[0] - 5, pos[1] - 5), (pos[0] + 5, pos[1] + 5), (0, 0, 255), 1)
        self._img = cv2.line(self._img, (pos[0] - 5, pos[1] + 5), (pos[0] + 5, pos[1] - 5), (0, 0, 255), 1)
    cv2.imwrite(self._dest_img_path, self._img)

if __name__ == '__main__':

  rospy.init_node('plotter')

  plotter = Plotter(rospy.get_param("~img_src", "/home/log/map.png"),
                    rospy.get_param("~img_dest", "/home/log/map_dest.png"),
                    rospy.get_param("~meters_per_pixel", 0.1),
                    rospy.get_param("~bearing_deg", 0.0),
                    rospy.get_param("~zero_px", [100, 100]),
                    rospy.get_param("~robot_track", '/l3xz/odom_recorder/track'),
                    rospy.get_param("~opi_tracks", ['/l3xz/thermal_recorder/track', '/l3xz/radiation_recorder/track']))
 
  rate = rospy.Rate(1)
  while not rospy.is_shutdown():
    plotter.plot()
    rate.sleep()
