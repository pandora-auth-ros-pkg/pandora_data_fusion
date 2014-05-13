// "Copyright [year] <Copyright Owner>"

#include "alert_handler/utils.h"

namespace pandora_data_fusion
{
namespace pandora_alert_handler
{

Point Utils::point2DAndHeight2Point3D(Point position, float height)
{
  position.z = height;
  return position;
}

float Utils::distanceBetweenPoints2D(Point a, Point b)
{
  float xDist = a.x - b.x;
  float yDist = a.y - b.y;

  return sqrt((xDist * xDist) + (yDist * yDist));
}

float Utils::distanceBetweenPoints3D(Point a, Point b)
{
  float xDist = a.x - b.x;
  float yDist = a.y - b.y;
  float zDist = a.z - b.z;

  return sqrt((xDist * xDist) + (yDist * yDist) + (zDist * zDist));
}

geometry_msgs::Quaternion Utils::calculateQuaternion(Point a, 
    Point b)
{
  tfScalar yaw;

  yaw = atan2(b.y - a.y, b.x - a.x);

  return tf::createQuaternionMsgFromRollPitchYaw(0, 0, yaw);
}

Point Utils::vector3ToPoint(tf::Vector3 vector)
{
  Point point;
  point.x = vector[0];
  point.y = vector[1];
  point.z = vector[2];

  return point;
}

bool Utils::arePointsInRange(Point pointA, Point pointB, float sensor_range )
{
  float dist = distanceBetweenPoints2D(pointA, pointB);

  if (dist > sensor_range)
    return false;
  else
    return true;
}

float Utils::probabilityFromStdDev(float boundingRadius, float deviation)
{
  if(deviation <= 0)
  {
    throw std::range_error("Standard deviation is a positive value.");
  }
  float x = boundingRadius / deviation;
  return 2 * (standardNormalIntegral(x) - x * standardNormal(x) - 0.5);
}

float Utils::stdDevFromProbability(float boundingRadius, float probability)
{
  if(probability > 1 || probability < 0)
  {
    throw std::range_error("Probability value is between 0 and 1.");
  }
  float x = 0.60, y = 0.60, epsilon = 0.0001, der = 0;
  do
  {
    x = y;
    der = -(pow(boundingRadius, 2) / pow(x, 4)) * 
      standardNormal(boundingRadius / x);
    y = x - (probabilityFromStdDev(boundingRadius, x) - probability) / der;
  }
  while(abs(y-x) >= epsilon);
  return y;
}

float Utils::standardNormal(float x)
{
  return (1/sqrt(2*PI)) * exp(-pow(x,2)/2);
}

float Utils::standardNormalIntegral(float x)
{
  return 0.5 * (1 + erf(x/sqrt(2)));
}

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

