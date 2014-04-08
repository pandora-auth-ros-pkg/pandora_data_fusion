// "Copyright [year] <Copyright Owner>"

#include "alert_handler/objects.h"

namespace pandora_data_fusion
{
namespace pandora_alert_handler
{

Object::Object() : matrixA_(1, 1), matrixB_(1, 1), matrixAB_(2), sysNoiseMu_(1),
            sysNoiseCov_(1), matrixH_(1, 1), measNoiseMu_(1), measNoiseCov_(1),
              priorMu_(1), priorCov_(1), input_(1)
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

void Object::initializeFilter()
{
  // System Model Initialization
  matrixA_(1, 1) = 1.0;
  matrixB_(1, 1) = 0.0;
  
  matrixAB_[0] = matrixA_;
  matrixAB_[1] = matrixB_;
  
  sysNoiseMu_(1) = 0.0;
  sysNoiseCov_(1, 1) = pow(0.2, 2);
  
  BFL::Gaussian systemUncertainty(sysNoiseMu_, sysNoiseCov_);
  
  sysPdf_.reset(new BFL::LinearAnalyticConditionalGaussian(matrixAB_,
                                                            systemUncertainty));
  sysModelX_.reset(
    new BFL::LinearAnalyticSystemModelGaussianUncertainty(&*sysPdf_));
  sysModelY_.reset(
    new BFL::LinearAnalyticSystemModelGaussianUncertainty(&*sysPdf_));
  sysModelZ_.reset(
    new BFL::LinearAnalyticSystemModelGaussianUncertainty(&*sysPdf_));
  
  // Measurement Model Initialization
  matrixH_(1, 1) = 1;
  
  measNoiseMu_(1) = 0.0;
  measNoiseCov_(1, 1) = pow(0.2, 2);
  
  BFL::Gaussian measurementUncertainty(measNoiseMu_, measNoiseCov_);
  
  measPdf_.reset(new BFL::LinearAnalyticConditionalGaussian(matrixH_,
                                                      measurementUncertainty));
  measModelX_.reset(
    new BFL::LinearAnalyticMeasurementModelGaussianUncertainty(&*measPdf_) );
  measModelY_.reset(
    new BFL::LinearAnalyticMeasurementModelGaussianUncertainty(&*measPdf_) );
  measModelZ_.reset(
    new BFL::LinearAnalyticMeasurementModelGaussianUncertainty(&*measPdf_) );
  
  // Priors
  priorMu_(1) = pose_.position.x;
  priorCov_(1, 1) = pow(0.5, 2);
  priorX_.reset(new BFL::Gaussian(priorMu_, priorCov_));
  filterX_.reset(new BFL::ExtendedKalmanFilter(&*priorX_));
  
  priorMu_(1) = pose_.position.y;
  priorCov_(1, 1) = pow(0.5, 2);
  priorY_.reset(new BFL::Gaussian(priorMu_, priorCov_));
  filterY_.reset(new BFL::ExtendedKalmanFilter(&*priorY_));
  
  priorMu_(1) = pose_.position.z;
  priorCov_(1, 1) = pow(0.5, 2);
  priorZ_.reset(new BFL::Gaussian(priorMu_, priorCov_));
  filterZ_.reset(new BFL::ExtendedKalmanFilter(&*priorZ_));
  
  // Input
  input_(1) = 0.0;
}

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

