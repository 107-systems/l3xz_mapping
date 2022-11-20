#ifndef POSE_FROM_TF_HPP
#define POSE_FROM_TF_HPP

#include <string>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <unistd.h>
#include <math.h>

class PoseLookup
{
public:

  /**
    @param readyCallback Function pointer to receive positional data if available.
    @param parent Parent frame name
    @param child Child frame name
   **/  
  PoseLookup(void(*readyCallback)(double dx, double dy, double dz, double r, double p, double y), 
             std::string parent = "base_link", std::string child = "road_frame");
  ~PoseLookup(){};

  /**
    @brief Blocking function waiting for availability of transform data. Will call readyCallback if once available.
    @param timeout_sec Interval for periodic availability recheck in seconds.
   **/
  void waitForIt(int timeout_sec = 3);

  /**
   @brief Get offset data if needed.
   @param dx x-Axis offset in m
   @param dy y-Axis offset in m
   @param dz z-Axis offset in m
   @param r roll in rad
   @param p pitch in rad
   @param y yaw in rad
   **/
  void getData(double* dx, double* dy, double* dz, double* r, double* p, double* y);

private:

  std::string parent, child;

  tf::TransformListener listener;
  tf::StampedTransform transform;

  double dx, dy, dz, r, p, y;

  /**
   @brief Pointer to callback function. Is called if waitForIt is done.
   @param dx x-Axis offset in m
   @param dy y-Axis offset in m
   @param dz z-Axis offset in m
   @param r roll in rad
   @param p pitch in rad
   @param y yaw in rad
   **/   
  void(*readyCallback)(double dx, double dy, double dz, double r, double p, double y);
};
#endif 
