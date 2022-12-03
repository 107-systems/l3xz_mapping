#ifndef POSE_FROM_TF_HPP
#define POSE_FROM_TF_HPP

#include <string>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <unistd.h>
#include <math.h>

#include <l3xz_mapping/pose.hpp>

class PoseLookup
{
public:

  PoseLookup(){}; 
  ~PoseLookup(){};

  Pose waitForIt(std::string parent = "", std::string child = "", int timeout_sec = 3);

private:

  std::string parent, child;

  tf::TransformListener listener;
  tf::StampedTransform transform;

  Pose _pose;
};
#endif 
