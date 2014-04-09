// "Copyright [year] <Copyright Owner>"

#ifndef ALERT_HANDLER_OBJECT_LIST_H
#define ALERT_HANDLER_OBJECT_LIST_H

#include <list>
#include <vector>
#include <boost/iterator/iterator_adaptor.hpp>

#include <bfl/filter/extendedkalmanfilter.h>
#include <bfl/model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <bfl/model/linearanalyticmeasurementmodel_gaussianuncertainty.h>
#include <bfl/pdf/analyticconditionalgaussian.h>
#include <bfl/pdf/linearanalyticconditionalgaussian.h>

#include "visualization_msgs/MarkerArray.h"

#include "alert_handler/objects.h"
// #include "alert_handler/const_iterator_const_ref.h"
#include "alert_handler/utils.h"

namespace pandora_data_fusion
{
namespace pandora_alert_handler
{

template <class ObjectType>
class ObjectList
{
 public:

  //!< Type Definitions
  typedef boost::shared_ptr< ObjectType > Ptr;
  typedef boost::shared_ptr< ObjectType const > ConstPtr;
  typedef std::list< Ptr > List;
  typedef typename List::iterator iterator;  
  typedef typename List::const_iterator const_iterator_vers_ref;
  typedef typename List::const_iterator const_iterator;
  // typedef const_iterator_const_ref<const_iterator_vers_ref, Ptr, 
            // ConstPtr> const_iterator;
  typedef std::list<iterator> IteratorList;

  typedef BFL::LinearAnalyticConditionalGaussian
    AnalyticGaussian;
  typedef boost::shared_ptr<AnalyticGaussian> AnalyticGaussianPtr;
  typedef BFL::LinearAnalyticSystemModelGaussianUncertainty
    SystemModel;
  typedef boost::shared_ptr<SystemModel> SystemModelPtr;
  typedef BFL::LinearAnalyticMeasurementModelGaussianUncertainty
    MeasurementModel;
  typedef boost::shared_ptr<MeasurementModel> MeasurementModelPtr;

 public:

  ObjectList(int counterThreshold = 1, float distanceThreshold = 0.5);

  const_iterator begin() const;
  const_iterator end() const;
  int size() const;
  bool isObjectPoseInList(const ObjectConstPtr& object, float closestAlert) const;

  bool add(const Ptr& object);
  void pop_back();
  void clear();

  void removeInRangeOfObject(const ObjectConstPtr& object, float range);

  void getObjectsPosesStamped(
    std::vector<geometry_msgs::PoseStamped>* poses) const;

  void fillGeotiff(
    data_fusion_communications::DatafusionGeotiffSrv::Response* res) const;

  void getVisualization(visualization_msgs::MarkerArray* markers) const;

  void setParams(int counterThreshold, float distanceThreshold);

 protected:
  
  /**
  @brief Initialize filter's pdf for the current object
  @return void
  **/
  void initializeFilterModel();

  bool isAnExistingObject(
    const ConstPtr& object, IteratorList* iteratorListPtr);

  void updateObject(
    const Ptr& object,
      const IteratorList& iteratorList);

  void removeElementAt(iterator it);

 protected:

  List objects_;
  float DIST_THRESHOLD;
  int COUNTER_THRES;

  //!< Filter's combined matrix
  std::vector<MatrixWrapper::Matrix> matrixAB_;
  //!< Filter's system pdf
  AnalyticGaussianPtr systemPdfPtr_;
  //!< Filter's system model for dimension x
  SystemModelPtr systemModelX_;
  //!< Filter's system model for dimension y
  SystemModelPtr systemModelY_;
  //!< Filter's system model for dimension z
  SystemModelPtr systemModelZ_;
  
  //!< Filter's measurement matrix H
  MatrixWrapper::Matrix matrixH_;
  //!< Filter's measurement pdf
  AnalyticGaussianPtr measurementPdfPtr_;
  //!< Filter's measurement model for dimension x
  MeasurementModelPtr measurementModelX_;
  //!< Filter's measurement model for dimension y
  MeasurementModelPtr measurementModelY_;
  //!< Filter's measurement model for dimension z
  MeasurementModelPtr measurementModelZ_;

 private:

  friend class ObjectListTest;

 private:

  int id_;

};

typedef boost::shared_ptr< ObjectList<Object> > ObjectListPtr;
typedef boost::shared_ptr< ObjectList<Hole> >  HoleListPtr;
typedef boost::shared_ptr< ObjectList<Qr> >  QrListPtr;
typedef boost::shared_ptr< ObjectList<Hazmat> > HazmatListPtr;
typedef boost::shared_ptr< ObjectList<Tpa> >  TpaListPtr;

typedef boost::shared_ptr< const ObjectList<Object> > ObjectListConstPtr;
typedef boost::shared_ptr< const ObjectList<Hole> >  HoleListConstPtr;
typedef boost::shared_ptr< const ObjectList<Qr> >  QrListConstPtr;
typedef boost::shared_ptr< const ObjectList<Hazmat> > HazmatListConstPtr;
typedef boost::shared_ptr< const ObjectList<Tpa> >  TpaListConstPtr;

template <class ObjectType>
ObjectList<ObjectType>::
ObjectList(int counterThreshold, float distanceThreshold) : matrixH_(1, 1)
{
  id_ = 0;
  COUNTER_THRES = counterThreshold;
  DIST_THRESHOLD = distanceThreshold;
  initializeFilterModel();
}

template <class ObjectType>
typename ObjectList<ObjectType>::const_iterator
  ObjectList<ObjectType>::begin() const
{
    return objects_.begin();
}

template <class ObjectType>
typename ObjectList<ObjectType>::const_iterator
  ObjectList<ObjectType>::end() const
{
    return objects_.end();
}

template <class ObjectType>
bool ObjectList<ObjectType>::add(const Ptr& object)
{
  IteratorList iteratorList;
  
  //!< Printing information about existing objects
  ROS_INFO("printing existing objects' positions");
  for (iterator it = objects_.begin(); it != objects_.end(); ++it)
  {
      ROS_INFO("x = %f", (*it)->getPose().position.x);
      ROS_INFO("y = %f", (*it)->getPose().position.y);
      ROS_INFO("z = %f", (*it)->getPose().position.z);
  }
  
  if (isAnExistingObject(object, &iteratorList))
  {
    ROS_INFO("updating existing object");
    updateObject(object, iteratorList);
    return false;
  }
  ROS_INFO("New object found");
  object->initializeObjectFilter();
  
  object->setId(id_++);
  objects_.push_back(object);
  return true;
}

template <class ObjectType>
void ObjectList<ObjectType>::removeElementAt(
  ObjectList<ObjectType>::iterator it)
{
    objects_.erase(it);
}

template <class ObjectType>
void ObjectList<ObjectType>::setParams(int counterThreshold,
    float distanceThreshold)
{
  COUNTER_THRES = counterThreshold;
  DIST_THRESHOLD = distanceThreshold;
}

template <class ObjectType>
int ObjectList<ObjectType>::size() const
{
  return objects_.size();
}

template <class ObjectType>
void ObjectList<ObjectType>::pop_back()
{
  objects_.pop_back();
}

template <class ObjectType>
void ObjectList<ObjectType>::clear()
{
  objects_.clear();
}

template <class ObjectType>
bool ObjectList<ObjectType>::isObjectPoseInList(
    const ObjectConstPtr& object, float range) const
{
  for (const_iterator it = this->begin(); it != this->end(); ++it)
  {
    float distance =
      Utils::distanceBetweenPoints3D(object->getPose().position,
                                      (*it)->getPose().position);
         
    if (distance < range)
    {
      return true;
    }
  }

  return false;
}

template <class ObjectType>
void ObjectList<ObjectType>::removeInRangeOfObject(
    const ObjectConstPtr& object, float range)
{
  iterator iter = objects_.begin();

  while (iter != objects_.end())
  {
    bool inRange = Utils::distanceBetweenPoints3D(
      object->getPose().position, (*iter)->getPose().position) < range;

    if ( inRange ) 
    {
      ROS_DEBUG_NAMED("object_handler",
         "[OBJECT_HANDLER %d] Deleting hole...", __LINE__);
      objects_.erase(iter++);
    }
    else 
    {
      ++iter;
    }
  }
}

template <class ObjectType>
void ObjectList<ObjectType>::getObjectsPosesStamped(
    std::vector<geometry_msgs::PoseStamped>* poses) const
{
  for (const_iterator it = this->begin(); it != this->end(); ++it)
  {
    poses->push_back((*it)->getPoseStamped());
  }
}

template <class ObjectType>
void ObjectList<ObjectType>::fillGeotiff(
    data_fusion_communications::DatafusionGeotiffSrv::Response* res) const
{
  for (const_iterator it = this->begin(); it != this->end(); ++it)
  {
    (*it)->fillGeotiff(res);
  }
}

template <class ObjectType>
void ObjectList<ObjectType>::getVisualization(
    visualization_msgs::MarkerArray* markers) const
{
  markers->markers.clear();
  for (const_iterator it = this->begin(); it != this->end(); ++it)
  {
    (*it)->getVisualization(markers);
  }
}

template <class ObjectType>
void ObjectList<ObjectType>::initializeFilterModel()
{
  //!< System Model Initialization
  //!< Filter's system matrix A
  MatrixWrapper::Matrix matrixA_(1, 1);
  //!< Filter's system matrix B
  MatrixWrapper::Matrix matrixB_(1, 1);
  //!< Filter's system noise mean
  MatrixWrapper::ColumnVector systemNoiseMu_(1);
  //!< Filter's system noise covariance
  MatrixWrapper::SymmetricMatrix systemNoiseVar_(1);

  matrixA_(1, 1) = 1.0;
  matrixB_(1, 1) = 0.0;
  
  matrixAB_.push_back(matrixA_);
  matrixAB_.push_back(matrixB_);
  
  systemNoiseMu_(1) = 0.0;
  systemNoiseVar_(1, 1) = pow(0.05, 2);
  
  BFL::Gaussian systemUncertainty(systemNoiseMu_, systemNoiseVar_); 
  systemPdfPtr_.reset( new AnalyticGaussian(matrixAB_, systemUncertainty) );

  systemModelX_.reset( new SystemModel(systemPdfPtr_.get()) );
  systemModelY_.reset( new SystemModel(systemPdfPtr_.get()) );
  systemModelZ_.reset( new SystemModel(systemPdfPtr_.get()) );

  //!< Measurement Model Initialization
  matrixH_(1, 1) = 1.0;
  //!< Filter's measurement noise mean
  MatrixWrapper::ColumnVector measurementNoiseMu_(1);
  //!< Filter's measurement noise covariance
  MatrixWrapper::SymmetricMatrix measurementNoiseVar_(1);
 
  measurementNoiseMu_(1) = 0.0;
  measurementNoiseVar_(1, 1) = pow(0.5, 2);
  
  BFL::Gaussian measurementUncertainty(measurementNoiseMu_, 
      measurementNoiseVar_);
  measurementPdfPtr_.reset( new AnalyticGaussian(matrixH_, 
      measurementUncertainty ));

  measurementModelX_.reset( new MeasurementModel(measurementPdfPtr_.get()) );
  measurementModelY_.reset( new MeasurementModel(measurementPdfPtr_.get()) );
  measurementModelZ_.reset( new MeasurementModel(measurementPdfPtr_.get()) );
}

template <class ObjectType>
bool ObjectList<ObjectType>::isAnExistingObject(
    const ConstPtr& object, IteratorList* iteratorListPtr)
{
  for (iterator it = objects_.begin(); it != objects_.end(); ++it)
  {
    if ((*it)->isSameObject(object, DIST_THRESHOLD))
    {
      iteratorListPtr->push_back(it);
    }
  }
  if (!iteratorListPtr->empty()) 
  {
    return true;
  }
  return false;
}

template <class ObjectType>
void ObjectList<ObjectType>::updateObject(
    const Ptr& object, const IteratorList& iteratorList)
{
  for ( typename IteratorList::const_iterator it = iteratorList.begin();
         it != iteratorList.end() ; ++it)
  {
    Point objectPosition = object->getPose().position;
    MatrixWrapper::ColumnVector measurement(1);
    
    //!< Printing object's most resest information.
    BFL::Pdf<MatrixWrapper::ColumnVector>* posterior = 
      (*(*it))->getFilterX()->PostGet();
    ROS_INFO("object's previous x position = %f", 
        posterior->ExpectedValueGet()(1));
    ROS_INFO("new object's x position = %f", objectPosition.x);
    posterior = (*(*it))->getFilterY()->PostGet();
    ROS_INFO("object's previous y position = %f", 
        posterior->ExpectedValueGet()(1));
    ROS_INFO("new object's y position = %f", objectPosition.y);
    posterior = (*(*it))->getFilterZ()->PostGet();
    ROS_INFO("object's previous z position = %f", 
        posterior->ExpectedValueGet()(1));
    ROS_INFO("new object's z position = %f", objectPosition.z);
    
    //!< Updating existing object's filter pdfs.
    measurement(1) = objectPosition.x;
    (*(*it))->getFilterX()->Update(systemModelX_.get(), 
        (*(*it))->getInput(), measurementModelX_.get(), measurement);
    
    measurement(1) = objectPosition.y;
    (*(*it))->getFilterY()->Update(systemModelY_.get(), 
        (*(*it))->getInput(), measurementModelY_.get(), measurement);
    
    measurement(1) = objectPosition.z;
    (*(*it))->getFilterZ()->Update(systemModelZ_.get(), 
        (*(*it))->getInput(), measurementModelZ_.get(), measurement);

    //!< Updating existing object's expected pose.
    Pose newObjectPose;
    newObjectPose.position.x = (*(*it))->getFilterX()->PostGet()
      ->ExpectedValueGet()(1);
    newObjectPose.position.y = (*(*it))->getFilterY()->PostGet()
      ->ExpectedValueGet()(1);
    newObjectPose.position.z = (*(*it))->getFilterZ()->PostGet()
      ->ExpectedValueGet()(1);

    //!< Printing object's current information.
    posterior = (*(*it))->getFilterX()->PostGet();
    ROS_INFO("object's new x position = %f", posterior->ExpectedValueGet()(1));
    ROS_INFO("object's new x covariance = %f", posterior->CovarianceGet()(1, 1));
    posterior = (*(*it))->getFilterY()->PostGet();
    ROS_INFO("object's new y position = %f", posterior->ExpectedValueGet()(1));
    ROS_INFO("object's new y covariance = %f", posterior->CovarianceGet()(1, 1));
    posterior = (*(*it))->getFilterZ()->PostGet();
    ROS_INFO("object's new z position = %f", posterior->ExpectedValueGet()(1));
    ROS_INFO("object's new z covariance = %f", posterior->CovarianceGet()(1, 1));

    //!< Updating existing object's orientation.
    newObjectPose.orientation = (*(*it))->getPose().orientation;

    (*(*it))->setPose(newObjectPose);

    bool setLegit = (*(*it))->getFilterX()->PostGet()->CovarianceGet()(1, 1)
                    < 0.04 &&
                    (*(*it))->getFilterY()->PostGet()->CovarianceGet()(1, 1)
                    < 0.04 &&
                    (*(*it))->getFilterZ()->PostGet()->CovarianceGet()(1, 1)
                    < 0.04;
    if (setLegit)
    {
      (*(*it))->setLegit(true);
    }
  }
}

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

#endif  // ALERT_HANDLER_OBJECT_LIST_H
