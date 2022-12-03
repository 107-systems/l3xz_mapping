#include <l3xz_mapping/tflistener.hpp>

Pose PoseLookup::waitForIt(std::string parent, std::string child, int timeout_sec)
{
  if(0 < parent.length() && 0 < child.length())
  {
          bool found = false;
          while(!found)
          {
            try{
              sleep(timeout_sec);
              listener.lookupTransform(parent, child, ros::Time(0), transform);
              tf::Matrix3x3 m(transform.getRotation());
              m.getRPY(_pose.roll, _pose.pitch, _pose.yaw);
              found = true;
              ROS_INFO("%s, %s\n", parent.c_str(), child.c_str());
              _pose.x = transform.getOrigin().x();
              _pose.y = transform.getOrigin().y();
              _pose.z = transform.getOrigin().z();
            }
            catch(tf::TransformException &ex)
            {
              ROS_WARN("Waiting for transform %s", ex.what());
            }
          }

  }
  return _pose;
}
