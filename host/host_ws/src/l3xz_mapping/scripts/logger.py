class Logger:
  
  def __init__(self, logfile, offset_time):

    self._coordinate_start = coordinate_start
    self._offset_time = offset_time
    self._logfile.open(logfile, "w")

  def __del__(self):
    
    self._logfile.close()

  def log(seconds, microseconds, utm_zone, utm_northing, utm_easting):

    timestamp = "{d}.{06d}".format(seconds, microseconds)
    self._logfile.write(timestamp + " " + str(utm_zone) + " " + str(utm_northing) + " " + str(utm_easting))
