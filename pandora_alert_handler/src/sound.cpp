// "Copyright [year] <Copyright Owner>"

#include "alert_handler/objects.h"

namespace pandora_data_fusion
{
namespace pandora_alert_handler
{

Sound::Sound()
{
  type_ = "sound";
}

PoseStamped Sound::getPoseStamped() const
{
  PoseStamped objPose = Object::getPoseStamped();
  objPose.header.frame_id = "sound_" + boost::to_string(id_);
  return objPose;
}

bool Sound::isSameObject(const ObjectConstPtr& object, float distance) const
{
  bool cond = false;
  
  if (!object->getType().compare(type_))
  {
    cond = Utils::distanceBetweenPoints2D(
        getPose().position, object->getPose().position) < distance;
  } 

  return cond;
}

void Sound::getVisualization(visualization_msgs::MarkerArray* markers) const
{
  visualization_msgs::Marker marker;

  marker.header.frame_id = "/world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "Sound";
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

