import rospy
import math
import cv2
import numpy as np
import os
import shutil
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import gpxpy
from geo import GeoPoint

class Artifact:

  def __init__(self, topic):
    self._topic = topic
    self._bridge = CvBridge()
    self._img = None
    self._sub = rospy.Subscriber(self._topic, Image, self._fill)

  @property
  def name(self):
    return self._topic

  def _fill(self, msg):
    try:
      self._img = self._bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
      print(e)

  def log(self, name):
    if self._img is not None:
      cv2.imwrite(name + ".png", self._img) 

class Logger:
  
  def __init__(self, logpath, artifacts = [], offset_time = 0):
    self._offset_time = offset_time
    if os.path.exists(logpath):
      shutil.rmtree(logpath)
    os.mkdir(logpath)
    self._logpath = logpath
    self._logfile = open(logpath + "/log.txt", "w")
    self._gpxfile = logpath + "/track.gpx"
    self._artifacts = []
    for a in artifacts:
      print(a)
      self._artifacts.append(Artifact(a))

    self._gpx = gpxpy.gpx.GPX()
    self._gpx_track = gpxpy.gpx.GPXTrack()
    self._gpx.tracks.append(self._gpx_track)
    self._gpx_segment = gpxpy.gpx.GPXTrackSegment()
    self._gpx_track.segments.append(self._gpx_segment)

  def __del__(self):
    self._logfile.close()

  def log(self, seconds, microseconds, point):
    utm_zone = point.utm_zone
    utm_northing = point.utm_northing
    utm_easting = point.utm_easting
    lat = point.latitude
    lon = point.longitude
    
    data = str(seconds) + "." + str(microseconds)[0:6] + " " + str(utm_zone) + " " + str(utm_northing) + " " + str(utm_easting)
    rospy.loginfo(data)
    self._logfile.write(data + "\n")

    self._gpx_segment.points.append(gpxpy.gpx.GPXTrackPoint(lat, lon))
    print(self._gpxfile)
    with open(self._gpxfile, "w") as f:
      f.write(self._gpx.to_xml())

    for artifact in self._artifacts:
      artifactpath = self._logpath + artifact.name
      if not os.path.exists(artifactpath):
        os.makedirs(artifactpath, exist_ok = True)
      artifactpath += "/" + data
      artifact.log(artifactpath)
