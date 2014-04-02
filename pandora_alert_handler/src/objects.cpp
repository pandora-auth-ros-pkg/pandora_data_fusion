// "Copyright [year] <Copyright Owner>"

#include "alert_handler/objects.h"

namespace pandora_data_fusion
{
namespace pandora_alert_handler
{

Object::Object() : A(3, 3), B(3, 3), AB(2), sysNoise_Mu(3),
      sysNoise_Cov(3), H(1, 3), measNoise_Mu(1), measNoise_Cov(1), prior_Mu(3),
        prior_Cov(2), input(3)
{
  counter_ = 0;
  legit_ = false;
  frame_id_ = "/world";
}

geometry_msgs::PoseStamped Object::getPoseStamped() const
{
  geometry_msgs::PoseStamped objPoseStamped;
  objPoseStamped.pose = pose_;
  return objPoseStamped;
}

bool Object::isSameObject(const ConstPtr& object, float distance) const
{
  return Utils::distanceBetweenPoints3D(
      pose_.position, object->getPose().position)
      < distance;
}

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

