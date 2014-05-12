// "Copyright [year] <Copyright Owner>"

#include "alert_handler/alert_handler.h"

namespace pandora_data_fusion
{
namespace pandora_alert_handler
{

AlertHandler::AlertHandler(const std::string& ns): nh_(ns)
{
  map_.reset( new Map );

  holes_.reset( new ObjectList<Hole> );
  qrs_.reset( new ObjectList<Qr> );
  hazmats_.reset( new ObjectList<Hazmat> );
  thermals_.reset( new ObjectList<Thermal> );
  faces_.reset( new ObjectList<Face> );
  motions_.reset( new ObjectList<Motion> );
  sounds_.reset( new ObjectList<Sound> );
  co2s_.reset( new ObjectList<Co2> );

  victimsToGo_.reset( new VictimList );
  victimsVisited_.reset( new VictimList );

  std::string mapType;
  nh_.getParam("map_type", mapType);
  objectFactory_.reset( new ObjectFactory(map_, mapType) );
  objectHandler_.reset( new ObjectHandler(holes_, qrs_, hazmats_, thermals_,
        faces_, motions_, sounds_, co2s_, victimsToGo_, victimsVisited_) );
  victimHandler_.reset( new VictimHandler(holes_ , thermals_,
        faces_, motions_, sounds_, co2s_, victimsToGo_, victimsVisited_) );

  initRosInterfaces();

  clientInitialize();
}

void AlertHandler::initRosInterfaces()
{
  std::string param; 

  //!< alert-concerned subscribers

  if (nh_.getParam("subscribed_topic_names/holeDirection", param))
  {
    holeDirectionSubscriber_ = nh_.subscribe(param, 
      1, &AlertHandler::holeDirectionAlertCallback, this);
  }
  else
  {
    ROS_FATAL("holeDirection topic name param not found");
    ROS_BREAK();
  }
  
  if (nh_.getParam("subscribed_topic_names/thermalDirection", param))
  {
    thermalDirectionSubscriber_ = nh_.subscribe(param, 
      1, &AlertHandler::objectDirectionAlertCallback< Thermal >, this);
  }
  else
  {
    ROS_FATAL("thermalDirection topic name param not found");
    ROS_BREAK();
  }
   
  if (nh_.getParam("subscribed_topic_names/qr", param))
  {
    qrSubscriber_ = nh_.subscribe(param, 1, &AlertHandler::qrAlertCallback, this);
  }
  else
  {
    ROS_FATAL("qr topic name param not found");
    ROS_BREAK();
  }
  
  if (nh_.getParam("subscribed_topic_names/hazmat", param))
  {
    hazmatSubscriber_ = nh_.subscribe(param, 
      1, &AlertHandler::hazmatAlertCallback, this);
  }
  else
  {
    ROS_FATAL("hazmat topic name param not found");
    ROS_BREAK();
  }

  if (nh_.getParam("subscribed_topic_names/faceDirection", param))
  {
    faceDirectionSubscriber_ = nh_.subscribe(param, 
      1, &AlertHandler::objectDirectionAlertCallback< Face >, this);
  }
  else
  {
    ROS_FATAL("faceDirection topic name param not found");
    ROS_BREAK();
  }

  if (nh_.getParam("subscribed_topic_names/co2Direction", param))
  {
    co2DirectionSubscriber_ = nh_.subscribe(param, 
      1, &AlertHandler::objectDirectionAlertCallback< Co2 >, this);
  }
  else
  {
    ROS_FATAL("co2Direction topic name param not found");
    ROS_BREAK();
  }

  if (nh_.getParam("subscribed_topic_names/motionDirection", param))
  {
    motionDirectionSubscriber_ = nh_.subscribe(param, 
      1, &AlertHandler::objectDirectionAlertCallback< Motion >, this);
  }
  else
  {
    ROS_FATAL("motionDirection topic name param not found");
    ROS_BREAK();
  }

  if (nh_.getParam("subscribed_topic_names/soundDirection", param))
  {
    soundDirectionSubscriber_ = nh_.subscribe(param, 
      1, &AlertHandler::objectDirectionAlertCallback< Sound >, this);
  }
  else
  {
    ROS_FATAL("soundDirection topic name param not found");
    ROS_BREAK();
  }

  //!< subscribers

  if (nh_.getParam("subscribed_topic_names/victimVerification", param))
  {
    victimVerificationSubscriber_ = nh_.subscribe(param,
      1, &AlertHandler::victimVerificationCallback, this);
  }
  else
  {
    ROS_FATAL("victimVerification topic name param not found");
    ROS_BREAK();
  }

  if (nh_.getParam("subscribed_topic_names/currentVictim", param))
  {
    currentVictimSubscriber_ = nh_.subscribe(param,
      1, &AlertHandler::selectedVictimCallback, this);
  }
  else
  {
    ROS_FATAL("currentVictim topic name param not found");
    ROS_BREAK();
  }

  if (nh_.getParam("subscribed_topic_names/map", param))
  {
    mapSubscriber_ = nh_.subscribe(param, 1, &AlertHandler::updateMap, this);
  }
  else
  {
    ROS_FATAL("map topic name param not found");
    ROS_BREAK();
  }

  //!< action servers

  if (nh_.getParam("action_server_names/get_victims", param))
  {
    victimsServer_.reset(new GetVictimsServer(nh_, param,  false));
  }
  else
  {
    ROS_FATAL("get_victims action name param not found");
    ROS_BREAK();
  }
  victimsServer_->registerGoalCallback(
    boost::bind( &AlertHandler::getVictimsCallback, this) );
  victimsServer_->start();

  if (nh_.getParam("action_server_names/delete_current_victim", param))
  {
    deleteVictimServer_.reset(new DeleteVictimServer(nh_, param,  false));
  } 
  else
  {
    ROS_FATAL("delete_current_victim action name param not found");
    ROS_BREAK();
  }
  deleteVictimServer_->registerGoalCallback(
    boost::bind( &AlertHandler::deleteVictimCallback, this) );
  deleteVictimServer_->start();

  if (nh_.getParam("action_server_names/validate_current_hole", param))
  {
    validateCurrentHoleServer_.reset(
    new ValidateCurrentHoleServer(nh_, param,  false));
  }
  else
  {
    ROS_FATAL("validate_current_hole action name param not found");
    ROS_BREAK();
  }
  validateCurrentHoleServer_->registerGoalCallback(
    boost::bind( &AlertHandler::validateCurrentHoleCallback, this) );
  validateCurrentHoleServer_->start();

  //!< service servers

  if (nh_.getParam("service_server_names/flush_queues", param))
  {
    flushService_ = nh_.advertiseService(param, 
        &AlertHandler::flushQueues, this);
  }
  else
  {
    ROS_FATAL("flush_queues service name param not found");
    ROS_BREAK();
  }

  if (nh_.getParam("service_server_names/get_objects", param))
  {
    getObjectsService_ = nh_.advertiseService(param, 
        &AlertHandler::getObjectsServiceCb, this);
  }
  else 
  {
    ROS_FATAL("get_objects service name param not found");
    ROS_BREAK();
  }

  if (nh_.getParam("service_server_names/get_markers", param))
  {
    getMarkersService_ = nh_.advertiseService(param, 
        &AlertHandler::getMarkersServiceCb, this);
  }
  else
  {
    ROS_FATAL("get_markers service name param not found");
    ROS_BREAK();
  }

  if (nh_.getParam("service_server_names/geotiff", param))
  {
    geotiffService_ = nh_.advertiseService(param, 
        &AlertHandler::geotiffServiceCb, this);
  }
  else
  {
    ROS_FATAL("geotiffSrv service name param not found");
    ROS_BREAK();
  }

  //!< dynamic reconfigure server

  dynReconfserver_.setCallback(boost::bind(
        &AlertHandler::dynamicReconfigCallback, this, _1, _2));

  //!< timers

  currentVictimTimer_ = nh_.createTimer(ros::Duration(0.1), 
      &AlertHandler::currentVictimTimerCb, this);
}  

//!< Alert-concerned callbacks

void AlertHandler::holeDirectionAlertCallback(
    const vision_communications::HolesDirectionsVectorMsg& msg)
{    
  ROS_DEBUG_NAMED("ALERT_HANDLER_ALERT_CALLBACK", "HOLE ALERT ARRIVED!");

  HolePtrVectorPtr holesVectorPtr;
  try
  {
    holesVectorPtr = objectFactory_->makeHoles(msg);
  }
  catch (AlertException ex)
  {
    ROS_ERROR("[ALERT_HANDLER %d]%s",  __LINE__, ex.what());
    return;
  }

  objectHandler_->handleHoles(holesVectorPtr, objectFactory_->getTransform());

  victimHandler_->notify();

}

void AlertHandler::hazmatAlertCallback(
    const vision_communications::HazmatAlertsVectorMsg& msg)
{
  ROS_DEBUG_NAMED("ALERT_HANDLER_ALERT_CALLBACK", "HAZMAT ALERT ARRIVED!");

  HazmatPtrVectorPtr hazmatsVectorPtr;
  try
  {
    hazmatsVectorPtr = objectFactory_->makeHazmats(msg);
  }
  catch (AlertException ex)
  {
    ROS_ERROR("[ALERT_HANDLER %d]%s",  __LINE__, ex.what());
    return;
  }

  objectHandler_->handleHazmats(hazmatsVectorPtr);

}

void AlertHandler::qrAlertCallback(
    const vision_communications::QRAlertsVectorMsg& msg)
{
  ROS_DEBUG_NAMED("ALERT_HANDLER_ALERT_CALLBACK", "QR ALERT ARRIVED!");  

  QrPtrVectorPtr qrsVectorPtr;
  try
  {
    qrsVectorPtr = objectFactory_->makeQrs(msg);
  }
  catch (AlertException ex)
  {
    ROS_ERROR("[ALERT_HANDLER %d]%s",  __LINE__, ex.what());
    return;
  }

  objectHandler_->handleQrs(qrsVectorPtr);

}

//!< Other Callbacks

void AlertHandler::currentVictimTimerCb(const ros::TimerEvent& event)
{
  tf::StampedTransform stampedTransform;

  if( victimHandler_->getCurrentVictimTransform(&stampedTransform) )
  {
    currentVictimBroadcaster_.sendTransform(stampedTransform);
  }
}

void AlertHandler::victimVerificationCallback(
    const data_fusion_communications::VictimVerificationMsg& msg)
{
}

void AlertHandler::selectedVictimCallback(const std_msgs::Int16& msg)
{
  victimHandler_->setCurrentVictimIndex(msg.data);
}

void AlertHandler::getVictimsCallback()
{
  victimsServer_->acceptNewGoal();

  data_fusion_communications::GetVictimsResult result;

  victimHandler_->getVictimsMsg( &result.victimsArray );

  victimsServer_->setSucceeded(result);
}

void AlertHandler::deleteVictimCallback()
{
  deleteVictimServer_->acceptNewGoal();

  victimHandler_->deleteCurrentVictim();

  deleteVictimServer_->setSucceeded();
}

void AlertHandler::validateCurrentHoleCallback()
{
  bool victimValid = validateCurrentHoleServer_->acceptNewGoal()->valid;

  victimHandler_->validateCurrentHole(victimValid);

  validateCurrentHoleServer_->setSucceeded();
}

void AlertHandler::updateMap(const nav_msgs::OccupancyGridConstPtr& msg)
{
  prevxMin = msg->info.origin.position.x / msg->info.resolution 
             + msg->info.width / 2;
  prevyMin = msg->info.origin.position.y / msg->info.resolution 
             + msg->info.height / 2;

  *map_ = *msg;
}

void AlertHandler::dynamicReconfigCallback(
    const ::pandora_alert_handler::AlertHandlerConfig& config, uint32_t level)
{
  objectFactory_->dynamicReconfigForward( config.occupiedCellThres,
    config.highThres, config.lowThres, config.approachDist,
    config.orientationCircle, config.orientationDist); 

  holes_->setParams(-1, config.holeMinimumDist,
      config.holeXVarThres, config.holeYVarThres, config.holeZVarThres,
      config.holePriorXSD, config.holePriorYSD, config.holePriorZSD,
      config.holeSystemNoiseSD, config.holeMeasNoiseSD);
  qrs_->setParams(config.qrScore, config.qrMinimumDist,
      config.qrXVarThres, config.qrYVarThres, config.qrZVarThres,
      config.qrPriorXSD, config.qrPriorYSD, config.qrPriorZSD,
      config.qrSystemNoiseSD, config.qrMeasNoiseSD);
  hazmats_->setParams(config.hazmatScore, config.hazmatMinimumDist,
      config.hazmatXVarThres, config.hazmatYVarThres, config.hazmatZVarThres,
      config.hazmatPriorXSD, config.hazmatPriorYSD, config.hazmatPriorZSD,
      config.hazmatSystemNoiseSD, config.hazmatMeasNoiseSD);
  thermals_->setParams(config.thermalScore, config.thermalMinimumDist,
      config.thermalXVarThres, config.thermalYVarThres, config.thermalZVarThres,
      config.thermalPriorXSD, config.thermalPriorYSD, config.thermalPriorZSD,
      config.thermalSystemNoiseSD, config.thermalMeasNoiseSD);

  objectHandler_->updateParams(config.sensorRange, config.clusterRadius);

  victimHandler_->updateParams(
    config.clusterRadius , config.sameVictimRadius,
    config.approachDist, config.victimUpdate , config.verificationProbability);
}

///////////////////////////////////////////////////////

bool AlertHandler::getObjectsServiceCb(
    data_fusion_communications::GetObjectsSrv::Request& rq,
      data_fusion_communications::GetObjectsSrv::Response &rs)
{
  holes_->getObjectsPosesStamped(&rs.holes);
  qrs_->getObjectsPosesStamped(&rs.qrs);
  hazmats_->getObjectsPosesStamped(&rs.hazmats);
  thermals_->getObjectsPosesStamped(&rs.thermals);

  victimHandler_->getVictimsPosesStamped(&rs.victimsToGo, &rs.victimsVisited, 
      &rs.approachPoints);

  return true;
}

bool AlertHandler::getMarkersServiceCb(
    data_fusion_communications::GetMarkersSrv::Request& rq,
      data_fusion_communications::GetMarkersSrv::Response &rs)
{
  holes_->getVisualization(&rs.holes);
  qrs_->getVisualization(&rs.hazmats);
  hazmats_->getVisualization(&rs.qrs);
  thermals_->getVisualization(&rs.thermals);

  victimHandler_->getVisualization(&rs.victimsVisited, &rs.victimsToGo);

  return true;
}

bool AlertHandler::geotiffServiceCb(
    data_fusion_communications::DatafusionGeotiffSrv::Request &req,
      data_fusion_communications::DatafusionGeotiffSrv::Response &res)
{
  qrs_->fillGeotiff(&res);
  hazmats_->fillGeotiff(&res);
  victimHandler_->fillGeotiff(&res);

  for (int i = 0; i < res.victimsx.size(); i++)
  {
    res.victimsx[i] -= prevxMin;
    res.victimsy[i] -= prevyMin;
  }

  for (int i = 0; i < res.qrx.size(); i++)
  {
    res.qrx[i] -=  prevxMin;
    res.qry[i] -=  prevyMin;
  }

  for (int i = 0; i < res.hazmatx.size(); i++)
  {
    res.hazmatx[i] -=  prevxMin;
    res.hazmaty[i] -=  prevyMin;
  }

  return true;
}

bool AlertHandler::flushQueues(
    std_srvs::Empty::Request& rq,
      std_srvs::Empty::Response &rs)
{
  ROS_INFO_NAMED("ALERT_HANDLER_FLUSH_SERVICE", "Flushing lists!");
  holes_->clear();
  qrs_->clear();
  hazmats_->clear();
  thermals_->clear();
  victimHandler_->flush();
  return true;
}

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

