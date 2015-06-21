/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, P.A.N.D.O.R.A. Team.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the P.A.N.D.O.R.A. Team nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors:
 *   Tsirigotis Christos <tsirif@gmail.com>
 *********************************************************************/

#ifndef ALERT_HANDLER_OBJECT_HANDLER_H
#define ALERT_HANDLER_OBJECT_HANDLER_H

#include <boost/shared_ptr.hpp>
#include <boost/utility.hpp>

#include <std_msgs/Int32.h>

#include "pandora_data_fusion_msgs/QrNotificationMsg.h"
#include "pandora_data_fusion_msgs/ObstacleInfo.h"

#include "alert_handler/objects.h"
#include "alert_handler/object_list.h"
#include "alert_handler/victim_list.h"

namespace pandora_data_fusion
{
namespace pandora_alert_handler
{

  //!< Type Definitions
  typedef boost::shared_ptr<ros::NodeHandle> NodeHandlePtr;

  /**
    * @brief Class which is responsible for keeping or ignoring
    * alerts, as well as placing them in their appropriate lists.
    */
  class ObjectHandler : private boost::noncopyable
  {
    public:
      /**
        * @brief constructor
        * @param nh [NodeHandlePtr const&] Alert Handler's node to register publishers
        * @param victimsToGoList [VictimListConstPtr const&]
        * list with victims to go - to be used in alert filtering
        * @param victimsVisited [VictimListConstPtr const&]
        * list with victims visited - to be used in alert filtering
        */
      ObjectHandler(
          const NodeHandlePtr& nh,
          const VictimListConstPtr& victimsToGoList,
          const VictimListConstPtr& victimsVisited);

      void handleHoles(const HolePtrVectorPtr& newHoles,
          const tf::Transform& transform);
      /**
        * @brief methods that handle alerts after their creation.
        * They try to keep those of interest and ignore the rest.
        * @param objectsPtr [ObjectType::PtrVectorPtr const&] vector with alerts
        * @return void
        */
      template <class ObjectType>
      void handleObjects(
          const typename ObjectType::PtrVectorPtr& objectsPtr,
          const tf::Transform& transform);

      /**
        * @brief parameter updating from dynamic reconfiguration
        * @param sensor_range [float] sensor's range defines maximum distance
        * one can have from the alert.
        * @param victim_cluster_radius [float] defines if an alert can be associated
        * with a victim in question [identification mode]
        * @return void
        */
      void updateParams(float sensor_range, float victim_cluster_radius);

    private:
      /**
        * @brief Alert filtering for holes.
        * @param objectsPtr [ObjectType::PtrVectorPtr const&] vector with newly
        * created object alerts
        * @param cameraTransform [tf::Transform const&] camera transform gives
        * current location
        * @return void
        */
      template <class ObjectType>
      void keepValidObjects(const typename ObjectType::PtrVectorPtr& objectsPtr,
          const tf::Transform& cameraTransform);
      template <class ObjectType>
        void keepValidVerificationObjects(
            const typename ObjectType::PtrVectorPtr& objectsPtr);

    private:
      ros::Publisher qrPublisher_;
      ros::Publisher scorePublisher_;
      ros::Publisher obstaclePublisher_;

      VictimListConstPtr victimsToGoList_;
      VictimListConstPtr victimsVisitedList_;

      int roboCupScore_;

      float SENSOR_RANGE;
      float VICTIM_CLUSTER_RADIUS;
  };

  template <class ObjectType>
  void ObjectHandler::keepValidObjects(
      const typename ObjectType::PtrVectorPtr& objectsPtr,
      const tf::Transform& transform)
  {
    tf::Vector3 origin = transform.getOrigin();
    geometry_msgs::Point framePosition = Utils::vector3ToPoint(origin);

    typename ObjectType::PtrVector::iterator iter = objectsPtr->begin();

    while (iter != objectsPtr->end())
    {
      bool invalid = !Utils::arePointsInRange((*iter)->getPose().position,
          framePosition, ObjectType::is3D, SENSOR_RANGE);

      if (invalid)
      {
        ROS_INFO_NAMED("ALERT_HANDLER",
            "[OBJECT_HANDLER %d] Deleting not valid object...", __LINE__);
        ROS_INFO_NAMED("ALERT_HANDLER",
            "[OBJECT_HANDLER %d] SENSOR_RANGE = %f", __LINE__, SENSOR_RANGE);
        iter = objectsPtr->erase(iter);
      }
      else
      {
        ++iter;
      }
    }
  }

  /**
    * @details keepValidVerificationObjects should not be called for
    * symbol pois (qr, hazmat, landoltc, datamatrix) as well as thermal
    */
  template <class ObjectType>
  void ObjectHandler::handleObjects(
      const typename ObjectType::PtrVectorPtr& newObjects,
      const tf::Transform& transform)
  {
    if (ObjectType::getObjectType() != VictimImage::getObjectType() &&
        ObjectType::getObjectType() != Hazmat::getObjectType() &&
        ObjectType::getObjectType() != Landoltc::getObjectType() &&
        ObjectType::getObjectType() != DataMatrix::getObjectType()) {
      keepValidVerificationObjects<ObjectType>(newObjects);
    }
    for (int ii = 0; ii < newObjects->size(); ++ii) {
      if (ObjectType::getList()->add(newObjects->at(ii))) {
        std_msgs::Int32 updateScoreMsg;
        roboCupScore_ += ObjectType::getObjectScore();
        updateScoreMsg.data = roboCupScore_;
        scorePublisher_.publish(updateScoreMsg);
      }
    }
  }

  template <>
  void ObjectHandler::handleObjects<Thermal>(
      const typename Thermal::PtrVectorPtr& newObjects,
      const tf::Transform& transform)
  {
    keepValidObjects<Thermal>(newObjects, transform);
    for (int ii = 0; ii < newObjects->size(); ++ii) {
      if (Thermal::getList()->add(newObjects->at(ii))) {
        std_msgs::Int32 updateScoreMsg;
        roboCupScore_ += Thermal::getObjectScore();
        updateScoreMsg.data = roboCupScore_;
        scorePublisher_.publish(updateScoreMsg);
      }
    }
  }

  template <>
  void ObjectHandler::handleObjects<Qr>(
      const typename Qr::PtrVectorPtr& newQrs,
      const tf::Transform& transform)
  {
    for (int ii = 0; ii < newQrs->size(); ++ii)
    {
      if (Qr::getList()->add(newQrs->at(ii)))
      {
        pandora_data_fusion_msgs::QrNotificationMsg newQrNofifyMsg;
        newQrNofifyMsg.header.stamp = newQrs->at(ii)->getTimeFound();
        newQrNofifyMsg.header.frame_id = newQrs->at(ii)->getFrameId();
        newQrNofifyMsg.x = newQrs->at(ii)->getPose().position.x;
        newQrNofifyMsg.y = newQrs->at(ii)->getPose().position.y;
        newQrNofifyMsg.content = newQrs->at(ii)->getContent();
        qrPublisher_.publish(newQrNofifyMsg);
        std_msgs::Int32 updateScoreMsg;
        roboCupScore_ += Qr::getObjectScore();
        updateScoreMsg.data = roboCupScore_;
        scorePublisher_.publish(updateScoreMsg);
      }
    }
  }

  template <>
  void ObjectHandler::handleObjects<Obstacle>(
      const typename Obstacle::PtrVectorPtr& newObstacles,
      const tf::Transform& transform)
  {
    for (int ii = 0; ii < newObstacles->size(); ++ii)
    {
      ObstaclePtr obstacleToSend;
      if (Obstacle::getList()->add(newObstacles->at(ii))) {
        ROS_DEBUG("[ObjectHandler %d] Found new obstacle!", __LINE__);
        obstacleToSend = newObstacles->at(ii);
      }
      else {
        ROS_DEBUG("[ObjectHandler %d] Fetching old obstacle", __LINE__);
        // Fetch object from list
      }
      // Create and send info message to navigation
      pandora_data_fusion_msgs::ObstacleInfo obstacleInfo;
      obstacleInfo.id = obstacleToSend->getId();
      obstacleInfo.obstacleFrameId = obstacleToSend->getFrameId();
      obstacleInfo.obstaclePose = obstacleToSend->getPoseStamped();
      obstacleInfo.probability = obstacleToSend->getProbability();
      obstacleInfo.length = obstacleToSend->getLength();
      obstacleInfo.width = obstacleToSend->getWidth();
      obstacleInfo.type = obstacleToSend->getObstacleType();
      obstacleInfo.valid = true;
      // Publish order for obstacle costmap
      obstaclePublisher_.publish(obstacleInfo);
    }
  }

  /**
    * @details Sound and CO2 pois give us spatial information in a 2d
    * surface on their sensors' tf frame's plane. We should not search for them
    * in a sphere but in a cylinder.
    */
  template <class ObjectType>
  void ObjectHandler::keepValidVerificationObjects(
      const typename ObjectType::PtrVectorPtr& objectsPtr)
  {
    typename ObjectType::PtrVector::iterator iter = objectsPtr->begin();

    while (iter != objectsPtr->end()) {
      bool valid = false;
      valid = victimsToGoList_->isObjectPoseInList(
          (*iter), VICTIM_CLUSTER_RADIUS);
      if (!valid)
      {
        ROS_DEBUG_NAMED("OBJECT_HANDLER",
            "[OBJECT_HANDLER %d] Deleting not valid object...", __LINE__);
        iter = objectsPtr->erase(iter);
      }
      else
      {
        ++iter;
      }
    }
  }

  typedef boost::scoped_ptr< ObjectHandler >  ObjectHandlerPtr;

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

#endif  // ALERT_HANDLER_OBJECT_HANDLER_H
