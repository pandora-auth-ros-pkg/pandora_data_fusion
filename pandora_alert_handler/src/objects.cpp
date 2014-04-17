// "Copyright [year] <Copyright Owner>"

#include "alert_handler/objects.h"

namespace pandora_data_fusion
{
namespace pandora_alert_handler
{

Object::Object()
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

void Object::initializeObjectFilter(float prior_x_sd, float prior_y_sd,
    float prior_z_sd)
{
  //!< Priors  
  //!< Filter's prior mean
  MatrixWrapper::ColumnVector priorMu_(1);
  //!< Filter's prior covariance
  MatrixWrapper::SymmetricMatrix priorVar_(1, 1);

  priorMu_(1) = pose_.position.x;
  priorVar_(1, 1) = pow(prior_x_sd, 2);
  priorX_.reset( new BFL::Gaussian(priorMu_, priorVar_) );
  filterX_.reset( new Filter(priorX_.get()) );
  
  priorMu_(1) = pose_.position.y;
  priorVar_(1, 1) = pow(prior_y_sd, 2);
  priorY_.reset( new BFL::Gaussian(priorMu_, priorVar_) );
  filterY_.reset( new Filter(priorY_.get()) );
  
  priorMu_(1) = pose_.position.z;
  priorVar_(1, 1) = pow(prior_z_sd, 2);
  priorZ_.reset( new BFL::Gaussian(priorMu_, priorVar_) );
  filterZ_.reset( new Filter(priorZ_.get()) );
}

/**
 * @details Updates this object's position according to new measurement's
 * position. This update is the result of the change in object's conviction
 * pdf on its position which is calculated from the given filter model
 * and the current measurement. The filter is an implementation of
 * Kalman Filter.
 */
void Object::update(const ConstPtr& measurement, const FilterModelConstPtr& model)
{
  Point measurementPosition = measurement->getPose().position;
  MatrixWrapper::ColumnVector newPosition(1);
  //!< Filter's input vector
  MatrixWrapper::ColumnVector input(1);  
  //!< Input is 0.0 as our actions doesn't change the world model.
  input(1) = 0.0;
 
  //!< Printing object's most resest information.
  BFL::Pdf<MatrixWrapper::ColumnVector>* posterior = 
    filterX_->PostGet();
  ROS_INFO("object's previous x position = %f", 
      posterior->ExpectedValueGet()(1));
  ROS_INFO("new object's x position = %f", measurementPosition.x);
  posterior = filterY_->PostGet();
  ROS_INFO("object's previous y position = %f", 
      posterior->ExpectedValueGet()(1));
  ROS_INFO("new object's y position = %f", measurementPosition.y);
  posterior = filterZ_->PostGet();
  ROS_INFO("object's previous z position = %f", 
      posterior->ExpectedValueGet()(1));
  ROS_INFO("new object's z position = %f", measurementPosition.z);
    
  //!< Updating existing object's filter pdfs.
  SystemModelPtrVector systemModels = model->getSystemModels();
  MeasurementModelPtrVector measurementModels = model->getMeasurementModels();
  
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

  //!< Printing object's current information.
  posterior = filterX_->PostGet();
  ROS_INFO("object's new x position = %f", posterior->ExpectedValueGet()(1));
  ROS_INFO("object's new x covariance = %f", posterior->CovarianceGet()(1, 1));
  posterior = filterY_->PostGet();
  ROS_INFO("object's new y position = %f", posterior->ExpectedValueGet()(1));
  ROS_INFO("object's new y covariance = %f", posterior->CovarianceGet()(1, 1));
  posterior = filterZ_->PostGet();
  ROS_INFO("object's new z position = %f", posterior->ExpectedValueGet()(1));
  ROS_INFO("object's new z covariance = %f", posterior->CovarianceGet()(1, 1));

  //!< Setting existing object's orientation.
  newObjectPose.orientation = pose_.orientation;

  pose_ = newObjectPose;
}

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

