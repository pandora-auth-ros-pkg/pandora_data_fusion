// "Copyright [year] <Copyright Owner>"

#include "alert_handler/object_factory.h"

namespace pandora_data_fusion
{
namespace pandora_alert_handler
{

ObjectFactory::ObjectFactory(const MapPtr& map, const std::string& mapType)
{
  poseFinder_.reset( new PoseFinder(map, mapType) );
}

HolePtrVectorPtr ObjectFactory::makeHoles(
    const vision_communications::HolesDirectionsVectorMsg& msg)
{
  currentTransform_ = poseFinder_->lookupTransformFromWorld( msg.header );

  HolePtrVectorPtr holesVectorPtr( new HolePtrVector );
  for (int ii = 0; ii < msg.holesDirections.size(); ++ii)
  {
    try
    {
      HolePtr newHole( new Hole );
      setUpHole( newHole, msg.holesDirections[ii] );
      holesVectorPtr->push_back( newHole );
    }
    catch (AlertException ex)
    {
      ROS_WARN_NAMED("ALERT_HANDLER",
        "[ALERT_HANDLER %d] %s", __LINE__, ex.what());
    }
  }

  return holesVectorPtr;
}

ThermalPtrVectorPtr ObjectFactory::makeThermals(
    const data_fusion_communications::ThermalDirectionAlertMsg& msg)
{
  currentTransform_ = poseFinder_->lookupTransformFromWorld( msg.header );

  ThermalPtrVectorPtr thermalsVectorPtr( new ThermalPtrVector );
  try
  {
    ThermalPtr newThermal( new Thermal );
    setUpThermal( newThermal, msg );
    thermalsVectorPtr->push_back( newThermal );
  }
  catch (AlertException ex)
  {
    ROS_WARN_NAMED("ALERT_HANDLER", "[ALERT_HANDLER %d] %s", __LINE__, ex.what());
  }

  return thermalsVectorPtr;
}

HazmatPtrVectorPtr ObjectFactory::makeHazmats(
    const vision_communications::HazmatAlertsVectorMsg& msg)
{
  currentTransform_ = poseFinder_->lookupTransformFromWorld( msg.header );

  HazmatPtrVectorPtr hazmatsVectorPtr( new HazmatPtrVector );
  for (int ii = 0; ii < msg.hazmatAlerts.size(); ++ii)
  {
    try
    {
      HazmatPtr newHazmat( new Hazmat );
      setUpHazmat( newHazmat, msg.hazmatAlerts[ii] );
      hazmatsVectorPtr->push_back( newHazmat );
    }
    catch (AlertException ex)
    {
      ROS_WARN_NAMED("ALERT_HANDLER",
        "[ALERT_HANDLER %d] %s", __LINE__, ex.what());
    }
  }

  return hazmatsVectorPtr;
}

QrPtrVectorPtr ObjectFactory::makeQrs(
    const vision_communications::QRAlertsVectorMsg& msg)
{
  currentTransform_ = poseFinder_->lookupTransformFromWorld( msg.header );

  QrPtrVectorPtr qrsVectorPtr( new QrPtrVector );
  for (int ii = 0; ii < msg.qrAlerts.size(); ++ii)
  {
    try
    {
      QrPtr newQr( new Qr );
      setUpQr( newQr, msg.qrAlerts[ii] );
      qrsVectorPtr->push_back( newQr );
    }
    catch (AlertException ex)
    {
      ROS_WARN_NAMED("ALERT_HANDLER",
        "[ALERT_HANDLER %d] %s", __LINE__, ex.what());
    }
  }

  return qrsVectorPtr;
}

void ObjectFactory::dynamicReconfigForward(float occupiedCellThres,
    float highThres, float lowThres, float approachDist,
    float orientationCircle, float orientationDist)
{
  poseFinder_->updateParams( occupiedCellThres, 
    highThres, lowThres, approachDist,
    orientationDist, orientationCircle
  );
}

void ObjectFactory::setUpHole(const HolePtr& holePtr, 
    const vision_communications::HoleDirectionMsg& msg)
{
  holePtr->setPose( poseFinder_->findAlertPose(msg.yaw, 
        msg.pitch, currentTransform_) );
  holePtr->setProbability( msg.probability );
  holePtr->setHoleId( msg.holeId );
}

void ObjectFactory::setUpThermal(const ThermalPtr& thermalPtr, 
    const data_fusion_communications::ThermalDirectionAlertMsg& msg)
{
  thermalPtr->setPose( poseFinder_->findAlertPose(msg.yaw, 
        msg.pitch, currentTransform_) );
  thermalPtr->setProbability( msg.probability );
}

void ObjectFactory::setUpHazmat(const HazmatPtr& hazmatPtr, 
    const vision_communications::HazmatAlertMsg& msg)
{
  hazmatPtr->setPose( poseFinder_->findAlertPose(msg.yaw, 
        msg.pitch, currentTransform_) );
  hazmatPtr->setPattern( msg.patternType );
}

void ObjectFactory::setUpQr(const QrPtr& qrPtr, 
    const vision_communications::QRAlertMsg& msg)
{
  qrPtr->setPose( poseFinder_->findAlertPose(msg.yaw, 
        msg.pitch, currentTransform_) );
  qrPtr->setContent( msg.QRcontent );
}

void ObjectFactory::setUpObject(const ObjectPtr& objectPtr, 
    const vision_communications::HoleDirectionMsg& msg)
{
  objectPtr->setPose( poseFinder_->findAlertPose(msg.yaw, 
        msg.pitch, currentTransform_) );
  objectPtr->setProbability( msg.probability );
}

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

