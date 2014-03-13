// "Copyright [year] <Copyright Owner>"
#ifndef PANDORA_ALERT_HANDLER_INCLUDE_ALERT_HANDLER_OBJECT_FACTORY_H_
#define PANDORA_ALERT_HANDLER_INCLUDE_ALERT_HANDLER_OBJECT_FACTORY_H_

#include <nav_msgs/OccupancyGrid.h>

#include <string>
#include <vector>

#include "vision_communications/HolesDirectionsVectorMsg.h"
#include "vision_communications/FaceDirectionMsg.h"
#include "vision_communications/QRAlertsVectorMsg.h"
#include "vision_communications/HazmatAlertsVectorMsg.h"
#include "vision_communications/HolesPositionsVectorMsg.h"
#include "data_fusion_communications/ThermalDirectionAlertMsg.h"

#include "alert_handler/pose_finder.h"
#include "alert_handler/objects.h"
#include "alert_handler/utils.h"

#include <boost/shared_ptr.hpp>

typedef nav_msgs::OccupancyGrid Map;
typedef nav_msgs::OccupancyGridPtr MapPtr;
typedef nav_msgs::OccupancyGridConstPtr MapConstPtr;

class ObjectFactory {

 public:

  ObjectFactory(const MapConstPtr& map, const std::string& mapType,
      float occupiedCellThres = 0.5,
      float heightHighThres = 1.2, float heightLowThres = 0,
      float approachDist = 0.5, int orientationDist = 20,
      int orientationCircle = 10);

  HolePtrVectorPtr makeHoles(
               const vision_communications::HolesDirectionsVectorMsg& msg);
  ObjectPtrVectorPtr makeHolePoses(
                const vision_communications::HolesPositionsVectorMsg& msg) {}
  ObjectPtrVectorPtr makeFaces(
                       const vision_communications::FaceDirectionMsg& msg) {}
  TpaPtrVectorPtr makeTpas(
          const data_fusion_communications::ThermalDirectionAlertMsg& msg);
  ObjectPtrVectorPtr makeMlxs(
          const data_fusion_communications::ThermalDirectionAlertMsg& msg) {}
  HazmatPtrVectorPtr makeHazmats(
                  const vision_communications::HazmatAlertsVectorMsg& msg);
  QrPtrVectorPtr makeQrs(
                      const vision_communications::QRAlertsVectorMsg& msg);

  const tf::Transform& getTransform() const {
    return currentTransform_;
  }

  void dynamicReconfigForward(float occupiedCellThres,
    float highThres, float lowThres, float approachDist,
    int orientationCircle, int orientationDist);

 private:

  /**
  @brief Sets this Object up according to the info from the Alert
  @param objectPtr [const ObjectPtr&] Pointer to Object variable to be filled
  @param msg [const ..._communications::...Msg&] Incoming ros message containing info
  @return void
  **/
  void setUpObject(const HolePtr& holePtr, 
                        const vision_communications::HoleDirectionMsg& msg);
  void setUpObject(const ObjectPtr& objectPtr, 
                        const vision_communications::HolePositionMsg& msg) {}
  void setUpObject(const ObjectPtr& objectPtr, 
                        const vision_communications::FaceDirectionMsg& msg) {}
  void setUpObject(const TpaPtr& tpaPtr, 
                        const data_fusion_communications::ThermalDirectionAlertMsg& msg);
  void setUpObject(const ObjectPtr& objectPtr, 
                        const vision_communications::HoleDirectionMsg& msg);
  void setUpObject(const HazmatPtr& hazmatPtr, 
                        const vision_communications::HazmatAlertMsg& msg);
  void setUpObject(const QrPtr& qrPtr, 
                        const vision_communications::QRAlertMsg& msg);

 private:

  tf::Transform currentTransform_;
  
  PoseFinderPtr poseFinder_;
};

typedef boost::scoped_ptr< ObjectFactory > ObjectFactoryPtr;

#endif // PANDORA_ALERT_HANDLER_INCLUDE_ALERT_HANDLER_OBJECT_FACTORY_H_
