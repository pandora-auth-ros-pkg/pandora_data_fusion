// "Copyright [year] <Copyright Owner>"

#include "alert_handler/objects.h"

namespace pandora_data_fusion
{
namespace pandora_alert_handler
{

Object::Object()
{
  legit_ = false;
  frame_id_ = "/world";
}

float Object::distanceThres_ = 0;
FilterModelPtr Object::modelPtr_ = FilterModelPtr();

PoseStamped Object::getPoseStamped() const
{
  PoseStamped objPoseStamped;
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
  MatrixWrapper::ColumnVector priorMean(1);
  //!< Filter's prior covariance
  MatrixWrapper::SymmetricMatrix priorVariance(1, 1);

  float stdDeviation = Utils::stdDevFromProbability(
      distanceThres_, probability_);
  priorVariance(1, 1) = pow(stdDeviation, 2);

  priorMean(1) = pose_.position.x;
  priorX_.reset( new BFL::Gaussian(priorMean, priorVariance) );
  filterX_.reset( new Filter(priorX_.get()) );
  
  priorMean(1) = pose_.position.y;
  priorY_.reset( new BFL::Gaussian(priorMean, priorVariance) );
  filterY_.reset( new Filter(priorY_.get()) );
  
  priorMean(1) = pose_.position.z;
  priorZ_.reset( new BFL::Gaussian(priorMean, priorVariance) );
  filterZ_.reset( new Filter(priorZ_.get()) );
}

/**
 * @details Updates this object's position according to new measurement's
 * position. This update is the result of the change in object's conviction
 * pdf on its position which is calculated from the given filter model
 * and the current measurement. The filter is an implementation of
 * Kalman Filter.
 */
void Object::update(const ConstPtr& measurement)
{
  Point measurementPosition = measurement->getPose().position;
  MatrixWrapper::ColumnVector newPosition(1);
  //!< Filter's input vector
  MatrixWrapper::ColumnVector input(1);  
  //!< Input is 0.0 as our actions doesn't change the world model.
  input(1) = 0.0;
 
  //!< Updating existing object's filter pdfs.
  SystemModelPtrVector systemModels;
  systemModels = modelPtr_->getSystemModels();
  MeasurementModelPtrVector measurementModels;
  measurementModels = modelPtr_->getMeasurementModels();
  
  newPosition(1) = measurementPosition.x;
  filterX_->Update(systemModels[0].get(), 
      input, measurementModels[0].get(), newPosition);
    
  newPosition(1) = measurementPosition.y;
  filterY_->Update(systemModels[1].get(), 
      input, measurementModels[1].get(), newPosition);
    
  newPosition(1) = measurementPosition.z;
  filterZ_->Update(systemModels[2].get(), 
      input, measurementModels[2].get(), newPosition);

  //!< Updating existing object's expected pose.
  Pose newObjectPose;
  newObjectPose.position.x = filterX_->PostGet()
    ->ExpectedValueGet()(1);
  newObjectPose.position.y = filterY_->PostGet()
    ->ExpectedValueGet()(1);
  newObjectPose.position.z = filterZ_->PostGet()
    ->ExpectedValueGet()(1);

  //!< Setting existing object's orientation.
  newObjectPose.orientation = pose_.orientation;

  pose_ = newObjectPose;
  
  //!< Updating object's probability.
  probability_ = (Utils::probabilityFromStdDev(distanceThres_, getStdDevX()) + 
      Utils::probabilityFromStdDev(distanceThres_, getStdDevY()) +
      Utils::probabilityFromStdDev(distanceThres_, getStdDevZ())) / 3;

}

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

