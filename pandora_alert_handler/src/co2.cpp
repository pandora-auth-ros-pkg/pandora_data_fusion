// "Copyright [year] <Copyright Owner>"

#include "alert_handler/objects.h"

namespace pandora_data_fusion
{
namespace pandora_alert_handler
{

Co2::Co2()
{
  type_ = "co2";
}

PoseStamped Co2::getPoseStamped() const
{
  PoseStamped objPose = Object::getPoseStamped();
  objPose.header.frame_id = "co2_" + boost::to_string(id_);
  return objPose;
}

bool Co2::isSameObject(const ObjectConstPtr& object, float distance) const
{
  bool cond = false;
  
  if (!object->getType().compare(type_))
  {
    cond = Object::isSameObject(object, distance);
  } 

  return cond;
}

void Co2::getVisualization(visualization_msgs::MarkerArray* markers) const
{
  visualization_msgs::Marker marker;

  marker.header.frame_id = "/world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "Co2";
  marker.id = id_;

  marker.pose = pose_;

  marker.type = visualization_msgs::Marker::SPHERE;

  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;

  marker.color.r = 1;
  marker.color.g = 0.65;
  marker.color.b = 0;
  marker.color.a = 0.7;

  markers->markers.push_back(marker);
}

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

