// "Copyright [year] <Copyright Owner>"

#ifndef ALERT_HANDLER_OBJECT_HANDLER_H
#define ALERT_HANDLER_OBJECT_HANDLER_H

#include <boost/shared_ptr.hpp>
#include <boost/utility.hpp>

#include <std_msgs/Int32.h>

#include "data_fusion_communications/QrNotificationMsg.h"

#include "alert_handler/object_list.h"

namespace pandora_data_fusion
{
namespace pandora_alert_handler
{

class ObjectHandler : private boost::noncopyable
{
 public:

  ObjectHandler(HoleListPtr holeListPtr, QrListPtr qrListPtr,
                HazmatListPtr hazmatListPtr, ThermalListPtr thermalListPtr,
                FaceListPtr faceListPtr, MotionListPtr motionListPtr,
                SoundListPtr soundListPtr, Co2ListPtr co2ListPtr);

  void handleHoles(const HolePtrVectorPtr& newHoles, 
      const tf::Transform& transform);
  void handleQrs(const QrPtrVectorPtr& newQrs, 
      const tf::Transform& transform);
  void handleHazmats(const HazmatPtrVectorPtr& newHazmats, 
      const tf::Transform& transform);
  void handleThermals(const ThermalPtrVectorPtr& newThermals, 
      const tf::Transform& transform);
  void handleFaces(const FacePtrVectorPtr& newFaces, 
      const tf::Transform& transform);
  void handleMotions(const MotionPtrVectorPtr& newMotions, 
      const tf::Transform& transform);
  void handleSounds(const SoundPtrVectorPtr& newSounds, 
      const tf::Transform& transform);
  void handleCo2s(const Co2PtrVectorPtr& newCo2s, 
      const tf::Transform& transform);

  void updateParams(float sensor_range);

 private:

  void keepValidHoles(const HolePtrVectorPtr& holesPtr,
     const tf::Transform& cameraTransform);

 private:

  ros::Publisher qrPublisher_;
  ros::Publisher scorePublisher_;

  QrListPtr qrListPtr_;
  HazmatListPtr hazmatListPtr_;
  HoleListPtr holeListPtr_;
  ThermalListPtr thermalListPtr_;
  FaceListPtr faceListPtr_;
  MotionListPtr motionListPtr_;
  SoundListPtr soundListPtr_;
  Co2ListPtr co2ListPtr_;

  int roboCupScore_;
  
  float SENSOR_RANGE;

};

typedef boost::scoped_ptr< ObjectHandler >  ObjectHandlerPtr;

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

#endif  // ALERT_HANDLER_OBJECT_HANDLER_H
