import rospy
import math
import cv2
import numpy as np
import os
import shutil
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

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
    self._artifacts = []
    for a in artifacts:
      print(a)
      self._artifacts.append(Artifact(a))

  def __del__(self):
    self._logfile.close()

  def log(self, seconds, microseconds, utm_zone, utm_northing, utm_easting):
    data = str(seconds) + ".{:06d}".format(microseconds) + " " + str(utm_zone) + " " + str(utm_northing) + " " + str(utm_easting)
    rospy.loginfo(data)
    self._logfile.write(data + "\n")
    for artifact in self._artifacts:
      artifactpath = self._logpath + artifact.name
      if not os.path.exists(artifactpath):
        os.makedirs(artifactpath, exist_ok = True)
      artifactpath += "/" + str(seconds) + ".{:06d}".format(microseconds) + "_" + str(utm_zone) + str(utm_northing) + "_" + str(utm_easting)
      artifact.log(artifactpath)
