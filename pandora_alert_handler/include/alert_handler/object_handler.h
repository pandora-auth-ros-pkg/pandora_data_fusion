// "Copyright [year] <Copyright Owner>"

#ifndef PANDORA_ALERT_HANDLER_INCLUDE_ALERT_HANDLER_OBJECT_HANDLER_H_
#define PANDORA_ALERT_HANDLER_INCLUDE_ALERT_HANDLER_OBJECT_HANDLER_H_

#include "alert_handler/object_list.h"
#include "data_fusion_communications/QrNotificationMsg.h"

class ObjectHandler {

 public:

  ObjectHandler(HoleListPtr holeListPtr, QrListPtr qrListPtr,
                HazmatListPtr hazmatListPtr, TpaListPtr tpaListPtr ,
                float sensorRange = 2.5,
                float qrClosestAlert = 0.5,
                float hazmatClosestalert = 0.5);

  void handleHoles(HolePtrVectorPtr newHoles, const tf::Transform& transform);
  void handleQrs(QrPtrVectorPtr newQrs,
   const tf::Transform& transform, bool eraseHoles);
  void handleHazmats(HazmatPtrVectorPtr newHazmats, const tf::Transform& transform);
  void handleTpas(TpaPtrVectorPtr newTpas, const tf::Transform& transform);

  bool isHoleQr(const HolePtr& hole);
  bool isHoleHazmat(const HolePtr& hole);

  void updateParams(float sensor_range,
     float qrClosestAlert, float hazmatClosestalert);

 private:

  void keepValidHoles(HolePtrVectorPtr holesPtr,
     const tf::Transform& cameraTransform);

 private:

  ros::Publisher qrPublisher_;

  QrListPtr qrListPtr_;
  HazmatListPtr hazmatListPtr_;
  HoleListPtr holeListPtr_;
  TpaListPtr tpaListPtr_;

  float SENSOR_RANGE;
  float QR_CLOSEST_ALERT;
  float HAZMAT_CLOSEST_ALERT;

};

typedef boost::scoped_ptr< ObjectHandler >  ObjectHandlerPtr;

#endif  // PANDORA_ALERT_HANDLER_INCLUDE_ALERT_HANDLER_OBJECT_HANDLER_H_
