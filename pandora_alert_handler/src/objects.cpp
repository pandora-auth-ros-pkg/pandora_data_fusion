// "Copyright [year] <Copyright Owner>"

#include "alert_handler/objects.h"

namespace pandora_data_fusion
{
namespace pandora_alert_handler
{

Object::Object() : input_(1)
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

void Object::initializeObjectFilter()
{
  //!< Priors  
  //!< Filter's prior mean
  MatrixWrapper::ColumnVector priorMu_(1);
  //!< Filter's prior covariance
  MatrixWrapper::SymmetricMatrix priorVar_(1, 1);

  priorMu_(1) = pose_.position.x;
  priorVar_(1, 1) = pow(0.5, 2);
  priorX_.reset( new BFL::Gaussian(priorMu_, priorVar_) );
  filterX_.reset( new Filter(priorX_.get()) );
  
  priorMu_(1) = pose_.position.y;
  priorVar_(1, 1) = pow(0.5, 2);
  priorY_.reset( new BFL::Gaussian(priorMu_, priorVar_) );
  filterY_.reset( new Filter(priorY_.get()) );
  
  priorMu_(1) = pose_.position.z;
  priorVar_(1, 1) = pow(0.5, 2);
  priorZ_.reset( new BFL::Gaussian(priorMu_, priorVar_) );
  filterZ_.reset( new Filter(priorZ_.get()) );
  
  //!< Input is 0.0 as our actions doesn't change the world model.
  input_(1) = 0.0;
}

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

