#ifndef POSE_HPP
#define POSE_HPP

#include <string>
#include <math.h>

/**
 * Coordinate position and orientation handler
 **/
class Pose
{
public:

 Pose();
 double x, y, z;
 double yaw, pitch, roll;

 /**
  * @brief Print information as string.
  * @return String information
  **/
 std::string str();

  /** 
   * @brief Add positions concerning position and orientation
   * @param p Second pose
   **/
 Pose operator+(const Pose& p);

 /** 
  * @brief Add positions concerning position and orientation
  * @param p Second pose
 **/
 Pose& operator+=(const Pose& p);

 /**
  * @brief Calculate distance in xy plane
  * @param p Second pose
  * @return distance in meters
  **/ 
 double dist2D(const Pose& p);

/**
 * @brief Calculate angle in xy plane
 * @param p Second pose
 * @return angle in radians
 **/ 
 double angle(const Pose& p);

/**
 * @brief Calculate x distance
 * @param p Second pose
 * @return distance in meters
 **/ 
 double dX(const Pose& p);

/**
 * @brief Calculate y distance
 * @param p Second pose
 * @return distance in meters
 **/ 
 double dY(const Pose& p);

/**
 * @brief Rotation in pitch
 * @param r Angle in radians
**/
void rotatePitch(double r);

 /**
 * @brief Rotation in roll
 * @param r Angle in radians
**/
void rotateRoll(double r);

/**
 * @brief Rotation in yaw
 * @param r Angle in radians
 **/
void rotateYaw(double r);
};
#endif
