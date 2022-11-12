#include <tflistener/poseFromTf.hpp>

PoseLookup::PoseLookup(void(*readyCallback)(double dx, double dy, double dz, double r, double p, double y),
            std::string parent, std::string child)
{
  this->readyCallback = readyCallback;
  this->parent = parent;
  this->child = child;
}

void PoseLookup::waitForIt(int timeout_sec)
{
  bool found = false;
  while(!found)
  {
    try{
      sleep(timeout_sec);
      listener.lookupTransform(parent, child, ros::Time(0), transform);
      tf::Matrix3x3 m(transform.getRotation());
      m.getRPY(r, p, y);
      found = true;
      ROS_INFO("%s, %s\n", parent.c_str(), child.c_str());
      dx = transform.getOrigin().x();
      dy = transform.getOrigin().y();
      dz = transform.getOrigin().z();
      if(readyCallback)
      {
        readyCallback(dx, dy, dz, r, p, y);
      }
    }
    catch(tf::TransformException &ex)
    {
      ROS_WARN("Waiting for transform %s", ex.what());
    }
  }
}

void PoseLookup::getData(double* dx, double* dy, double* dz, double* r, double* p, double* y)
{
  *dx = this->dx;
  *dy = this->dy;
  *dz = this->dz;
  *r = this->r;
  *p = this->p;
  *y = this->y;
}

