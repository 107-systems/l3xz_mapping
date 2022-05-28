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
import numpy as np
import shutil

from nav_msgs.msg import OccupancyGrid
from l3xz_mapping.msg import Waypoint, Track

class Plotter:

  def __init__(self, src_img, dest_img, resolution, bearing_offset, start, track_topic, opi_topics = [], from_grid = False, grid_topic = "/grid"):
    self._from_grid = from_grid
    self._iteration = 0
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
    
    if not self._from_grid:
      self._img = cv2.imread(src_img)
    else:
      if os.path.exists(self._dest_img_path):
        shutil.rmtree(self._dest_img_path)
      os.mkdir(self._dest_img_path)
      self._grid_subscriber = rospy.Subscriber(grid_topic, OccupancyGrid, self._update_grid, queue_size = 1)

  def _update_opi(self, opi):
      for o in self._opis:
        if o.description == opi.description:
          o = opi
          return
      self._opis.append(opi)
          
  def _update_track(self, track):
    self._track = track

  def _update_grid(self, grid):

    width = grid.info.width;
    height = grid.info.height;
   
    self._img = np.zeros((height,width,3), np.uint8)

    for y in range(0, height):
      for x in range(0, width):
        idxOut = 3 * (x + y * width);
        idxIn = (x + (height - y - 1) * width);

        if -1 == grid.data[idxIn]:
          self._img[y, x] = [50, 0, 0]
        else:
          self._img[y, x] = [0, int(255 * grid.data[idxIn] * 0.01), 0]
    self._resolution = grid.info.resolution
    pose = grid.info.origin
    self._start = [int(-pose.position.x / self._resolution), int(-pose.position.y / self._resolution)]
    self.plot()

  def _position(self, x, y):
    self._height = self._img.shape[0]
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
    if not self._from_grid:
      cv2.imwrite(self._dest_img_path, self._img)
    else:
      cv2.imwrite(self._dest_img_path + "/" + str(self._iteration) + ".png", self._img)
      self._iteration += 1

if __name__ == '__main__':

  rospy.init_node('plotter')

  from_grid = rospy.get_param("~from_grid", False)

  plotter = Plotter(rospy.get_param("~img_src", "/home/log/map.png"),
                    rospy.get_param("~img_dest", "/home/log/map_dest.png"),
                    rospy.get_param("~meters_per_pixel", 0.1),
                    rospy.get_param("~bearing_deg", 0.0),
                    rospy.get_param("~zero_px", [100, 100]),
                    rospy.get_param("~robot_track", '/l3xz/odom_recorder/track'),
                    rospy.get_param("~opi_tracks", ['/l3xz/thermal_recorder/track', '/l3xz/radiation_recorder/track']),
                    from_grid,
                    rospy.get_param("~grid_topic", "/grid"))
 
  rate = rospy.Rate(1)
  while not rospy.is_shutdown():
    if not from_grid:
      plotter.plot()
    rate.sleep()
