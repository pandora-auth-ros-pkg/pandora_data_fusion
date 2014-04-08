// "Copyright [year] <Copyright Owner>"

#ifndef ALERT_HANDLER_UTILS_H
#define ALERT_HANDLER_UTILS_H

#include <utility>
#include <boost/utility.hpp>

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <std_msgs/Empty.h>
#include <geometry_msgs/Pose.h>

#include "alert_handler/exceptions.h"
#include "alert_handler/defines.h"

class Utils : private boost::noncopyable
{
 public:

  static Point point2DAndHeight2Point3D(Point position, float height);
  static float distanceBetweenPoints2D(Point a, Point b);
  static float distanceBetweenPoints3D(Point a, Point b);
  static bool arePointsInRange(Point pointA, Point pointB, float sensor_range );
  static geometry_msgs::Quaternion calculateQuaternion(Point a,
    Point b);
  static Point vector3ToPoint(tf::Vector3 vector);

};

#endif  // ALERT_HANDLER_UTILS_H
