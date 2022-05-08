class Logger:
  
  def __init__(self, logfile, offset_time = 0):
    self._offset_time = offset_time
    self._logfile = open(logfile, "w")

  def __del__(self):
    self._logfile.close()

  def log(self, seconds, microseconds, utm_zone, utm_northing, utm_easting):
    self._logfile.write(str(seconds) + ".{:06d}".format(microseconds) + " " + str(utm_zone) + " " + str(utm_northing) + " " + str(utm_easting) + "\n")
