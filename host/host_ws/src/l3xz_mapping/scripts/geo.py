#!/usr/bin/python3
#-*- coding: utf-8 -*-
import math
import geopy
import geopy.distance
import utm

class GeoPoint:

  def __init__(self):
    self._latitude = None
    self._longitude = None
    self._utm_zone = None
    self._utm_easting = None
    self._utm_northing = None

  def __init__(self, latitude, longitude):
    self._latitude = latitude
    self._longitude = longitude
    self._update_utm()

  def __str__(self):
    return str(self.latitude) + ", " + str(self.longitude) + "\n" + self._utm_zone + " " + str(self._utm_easting) + " " + str(self._utm_northing)

  @property
  def latitude(self):
    return self._latitude
    self._longitude = value
    self._update_utm()
  
  @latitude.setter
  def latitude(self, value):
    self._latitude = value
    self._update_utm()

  @property
  def longitude(self):
    return self._longitude

  @longitude.setter
  def longitude(self, value):
    self._longitude = value
  
  @property
  def utm_zone(self):
    return self._utm_zone

  @property
  def utm_easting(self):
    return self._utm_easting

  @property
  def utm_northing(self):
    return self._utm_northing

  def _update_utm(self):
    u = utm.from_latlon(self._latitude, self._longitude)
    self._utm_easting = u[0]
    self._utm_northing = u[1]
    self._utm_zone = str(u[2]) + str(u[3])

  def point_from_delta(self, delta_y, delta_x, bearing_deg = 0):
    d = geopy.distance.geodesic(kilometers = (math.sqrt(delta_x ** 2 + delta_y ** 2) / 1000))
    bearing = 180 + bearing_deg + math.atan2(delta_y, delta_x) / math.pi * 180
    dest = d.destination(point = geopy.Point(self._latitude, self._longitude), bearing = bearing)
    return GeoPoint(dest.latitude, dest.longitude)
