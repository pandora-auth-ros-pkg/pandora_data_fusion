// "Copyright [year] <Copyright Owner>"

#include "alert_handler/objects.h"

namespace pandora_data_fusion
{
namespace pandora_alert_handler
{

Object::Object() : matrixA_(3, 3), matrixB_(3, 3), matrixAB_(2), 
  sysNoiseMu_(3), sysNoiseCov_(3), matrixH_(1, 3), measNoiseMu_(1), 
  measNoiseCov_(1), priorMu_(3), priorCov_(2), input_(3)
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

void Object::initializeFilter() {
  
  // System Model Initialization
  matrixA_(1,1) = 1.0;
  matrixA_(1,2) = 0.0;
  matrixA_(1,3) = 0.0;
  matrixA_(2,1) = 0.0;
  matrixA_(2,2) = 1.0;
  matrixA_(2,3) = 0.0;
  matrixA_(3,1) = 0.0;
  matrixA_(3,2) = 0.0;
  matrixA_(3,3) = 1.0;
  matrixB_(1,1) = 0.0;
  matrixB_(1,2) = 0.0;
  matrixB_(1,3) = 0.0;
  matrixB_(2,1) = 0.0;
  matrixB_(2,2) = 0.0;
  matrixB_(2,3) = 0.0;
  matrixB_(3,1) = 0.0;
  matrixB_(3,2) = 0.0;
  matrixB_(3,3) = 0.0;

  matrixAB_[0] = matrixA_;
  matrixAB_[1] = matrixB_;
  
  sysNoiseMu_(1) = 0.0;
  sysNoiseMu_(2) = 0.0;
  sysNoiseMu_(2) = 0.0;

  sysNoiseCov_ = 0.0;
  sysNoiseCov_(1,1) = pow(0.01,2);
  sysNoiseCov_(1,2) = 0.0;
  sysNoiseCov_(1,3) = 0.0;
  sysNoiseCov_(2,1) = 0.0;
  sysNoiseCov_(2,2) = pow(0.01,2);
  sysNoiseCov_(2,3) = 0.0;
  sysNoiseCov_(3,1) = 0.0;
  sysNoiseCov_(3,2) = 0.0;
  sysNoiseCov_(3,3) = pow(0.01,2);

  BFL::Gaussian systemUncertainty(sysNoiseMu_, sysNoiseCov_);

  BFL::LinearAnalyticConditionalGaussian sysPdf(matrixAB_, systemUncertainty);
  sysModel_ = new BFL::LinearAnalyticSystemModelGaussianUncertainty(&sysPdf);
  
  // Measurement Model Initialization
  matrixH_(1,1) = 1;
  matrixH_(1,2) = 1;
  matrixH_(1,3) = 1;
  
  measNoiseMu_(1) = 0;

  measNoiseCov_(1,1) = pow(0.05,2);
  BFL::Gaussian measurementUncertainty(measNoiseMu_, measNoiseCov_);

  BFL::LinearAnalyticConditionalGaussian measPdf(matrixH_, measurementUncertainty);
  measModel_ =
          new BFL::LinearAnalyticMeasurementModelGaussianUncertainty(&measPdf);
  
  // Prior
  priorMu_(1) = pose_.position.x;
  priorMu_(2) = pose_.position.y;
  priorMu_(3) = pose_.position.z;
  priorCov_(1,1) = pow(0.2,2);
  priorCov_(1,2) = 0.0;
  priorCov_(1,3) = 0.0;
  priorCov_(2,1) = 0.0;
  priorCov_(2,2) = pow(0.2,2);
  priorCov_(2,3) = 0.0;
  priorCov_(3,1) = 0.0;
  priorCov_(3,2) = 0.0;
  priorCov_(3,3) = pow(0.2,2);
  BFL::Gaussian prior(priorMu_, priorCov_);
  
  filter_ = new BFL::ExtendedKalmanFilter(&prior);
  
  // Input
  input_(1) = 0.0;
  input_(2) = 0.0;
  input_(3) = 0.0;
}

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

