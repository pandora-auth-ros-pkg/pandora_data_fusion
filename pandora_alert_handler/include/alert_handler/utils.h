// "Copyright [year] <Copyright Owner>"

#ifndef PANDORA_ALERT_HANDLER_INCLUDE_ALERT_HANDLER_UTILS_H_
#define PANDORA_ALERT_HANDLER_INCLUDE_ALERT_HANDLER_UTILS_H_

#include "ros/ros.h"
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>
#include "alert_handler/exceptions.h"
#include "alert_handler/defines.h"
#include "std_msgs/Empty.h"

typedef geometry_msgs::Pose Pose;
typedef geometry_msgs::Point Point;

class Utils {
 private:
  Utils() {}

 public:

  /* redundant ??
  static float pixelCoords2Meters(int pixels);
  static int meters2PixelCoords(float meters);
  */
  static Point point2DAndHeight2Point3D(Point position, float height);
  static float distanceBetweenPoints2D(Point a, Point b);
  static float distanceBetweenPoints3D(Point a, Point b);
  static bool arePointsInRange(Point pointA, Point pointB, float sensor_range );

  static geometry_msgs::Quaternion calculateQuaternion(Point a,
    Point b);
  static Point vector3ToPoint(tf::Vector3 vector);

};

#endif  // PANDORA_ALERT_HANDLER_INCLUDE_ALERT_HANDLER_UTILS_H_
