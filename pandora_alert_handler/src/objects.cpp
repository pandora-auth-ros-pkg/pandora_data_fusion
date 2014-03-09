// "Copyright [year] <Copyright Owner>"

#include "alert_handler/objects.h"

Object::Object() {
  counter_ = 0;
  legit_ = false;
  frame_id_ = "/world";
}

geometry_msgs::PoseStamped Object::getPoseStamped() const {

  geometry_msgs::PoseStamped objPoseStamped;
  objPoseStamped.pose = pose_;
  return objPoseStamped;
}

bool Object::isSameObject(const ConstPtr& object, float distance) const {

  return 
    Utils::distanceBetweenPoints3D(pose_.position, object->getPose().position)
      < distance;

}

