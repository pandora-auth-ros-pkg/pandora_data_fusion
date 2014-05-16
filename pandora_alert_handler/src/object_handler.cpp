// "Copyright [year] <Copyright Owner>"

#include "alert_handler/object_handler.h"

namespace pandora_data_fusion
{
  namespace pandora_alert_handler
  {

    ObjectHandler::ObjectHandler(HoleListPtr holeListPtr, QrListPtr qrListPtr, 
        HazmatListPtr hazmatListPtr, ThermalListPtr thermalListPtr, 
        FaceListPtr faceListPtr, MotionListPtr motionListPtr,
        SoundListPtr soundListPtr, Co2ListPtr co2ListPtr,
        const VictimListConstPtr& victimsToGoList,
        const VictimListConstPtr& victimsVisitedList) : 
      holeListPtr_(holeListPtr),
      qrListPtr_(qrListPtr),
      hazmatListPtr_(hazmatListPtr),
      thermalListPtr_(thermalListPtr),
      faceListPtr_(faceListPtr),
      motionListPtr_(motionListPtr),
      soundListPtr_(soundListPtr),
      co2ListPtr_(co2ListPtr),
      victimsToGoList_(victimsToGoList),
      victimsVisitedList_(victimsVisitedList)
    {
      Hole::setType("hole");
      Hazmat::setType("hazmat");
      Qr::setType("qr");
      Thermal::setType("thermal");
      Face::setType("face");
      Motion::setType("motion");
      Sound::setType("sound");
      Co2::setType("co2");

      Victim::setType("victim");

      roboCupScore_ = 0;

      std::string param;

      if (ros::param::get("published_topic_names/qr_notification", param))
      {
        qrPublisher_ = ros::NodeHandle().
          advertise<data_fusion_communications::QrNotificationMsg>(param, 10);
      }
      else
      {
        ROS_FATAL("qr_notification topic name param not found");
        ROS_BREAK();
      }

      if (ros::param::get("published_topic_names/robocup_score", param))
      {
        scorePublisher_ = ros::NodeHandle().
          advertise<std_msgs::Int32>(param, 10);
      }
      else
      {
        ROS_FATAL("robocup_score topic name param not found");
        ROS_BREAK();
      }
    }

    void ObjectHandler::handleHoles(const HolePtrVectorPtr& newHoles,
        const tf::Transform& transform)
    {
      keepValidHoles(newHoles, transform);

      for(int ii = 0; ii < newHoles->size(); ++ii)
      {
        holeListPtr_->add( newHoles->at(ii) );
      }
    }

    void ObjectHandler::handleQrs(const QrPtrVectorPtr& newQrs) 
    {
      for(int ii = 0; ii < newQrs->size(); ++ii)
      {
        int qrScore = qrListPtr_->add( newQrs->at(ii) );
        if(qrScore)
        {
          data_fusion_communications::QrNotificationMsg newQrNofifyMsg;
          newQrNofifyMsg.header.stamp = ros::Time::now();
          newQrNofifyMsg.x = newQrs->at(ii)->getPose().position.x;
          newQrNofifyMsg.y = newQrs->at(ii)->getPose().position.y;
          newQrNofifyMsg.content = newQrs->at(ii)->getContent();
          qrPublisher_.publish(newQrNofifyMsg);
          std_msgs::Int32 updateScoreMsg;
          roboCupScore_ += qrScore;
          updateScoreMsg.data = roboCupScore_;
          scorePublisher_.publish(updateScoreMsg);
        }
      }
    }

    int ObjectHandler::addToList(const ObjectPtr& newObject)
    {
      if(newObject->getType() == "thermal")
        return thermalListPtr_->add(
            boost::dynamic_pointer_cast<Thermal>(newObject));
      if(newObject->getType() == "hazmat")
        return hazmatListPtr_->add(
            boost::dynamic_pointer_cast<Hazmat>(newObject));
      if(newObject->getType() == "face")
        return faceListPtr_->add(
            boost::dynamic_pointer_cast<Face>(newObject));
      if(newObject->getType() == "motion")
        return motionListPtr_->add(
            boost::dynamic_pointer_cast<Motion>(newObject));
      if(newObject->getType() == "sound")
        return soundListPtr_->add(
            boost::dynamic_pointer_cast<Sound>(newObject));
      if(newObject->getType() == "co2")
        return co2ListPtr_->add(
            boost::dynamic_pointer_cast<Co2>(newObject));
    }

    void ObjectHandler::keepValidHoles(const HolePtrVectorPtr& holesPtr,
        const tf::Transform& transform)
    {
      tf::Vector3 origin = transform.getOrigin();
      geometry_msgs::Point framePosition = Utils::vector3ToPoint(origin);

      HolePtrVector::iterator iter = holesPtr->begin();

      while(iter != holesPtr->end())
      {
        bool invalid = !Utils::arePointsInRange((*iter)->getPose().position,
            framePosition, SENSOR_RANGE );

        if(invalid)
        {
          ROS_DEBUG_NAMED("object_handler",
              "[OBJECT_HANDLER %d] Deleting not valid hole...", __LINE__);
          iter = holesPtr->erase(iter);
        }
        else
        {
          ++iter;
        }
      }
    }

    void ObjectHandler::updateParams(float sensor_range, float victim_cluster_radius)
    {
      SENSOR_RANGE = sensor_range;
      VICTIM_CLUSTER_RADIUS = victim_cluster_radius;
    }

  }  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

