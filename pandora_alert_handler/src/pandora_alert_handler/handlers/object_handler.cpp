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

#include <string>

#include "pandora_data_fusion_msgs/WorldModel.h"

#include "pandora_alert_handler/handlers/object_handler.h"

namespace pandora_data_fusion
{
namespace pandora_alert_handler
{

  ObjectHandler::ObjectHandler(const ros::NodeHandlePtr& nh,
      const VictimListConstPtr& victimsToGoList,
      const VictimListConstPtr& victimsVisitedList,
      const QrListPtr& qrsToGo,
      const QrListPtr& qrsVisited) :
    victimsToGoList_(victimsToGoList),
    victimsVisitedList_(victimsVisitedList),
    qrsToGoList_(qrsToGo),
    qrsVisitedList_(qrsVisited)
  {
    std::string param;

    roboCupScore_ = 0;

    if (nh->getParam("published_topic_names/qr_info", param))
    {
      qrPublisher_ = nh->advertise<pandora_data_fusion_msgs::QrInfo>(param, 2);
    }
    else
    {
      ROS_FATAL("[ObjectHandler %d] qr_info topic name param not found", __LINE__);
      ROS_BREAK();
    }

    if (nh->getParam("published_topic_names/robocup_score", param))
    {
      scorePublisher_ = nh->advertise<std_msgs::Int32>(param, 2);
    }
    else
    {
      ROS_FATAL("[ObjectHandler %d] robocup_score topic name param not found", __LINE__);
      ROS_BREAK();
    }

    if (nh->getParam("published_topic_names/obstacle_info", param))
    {
      obstaclePublisher_ = nh->advertise<pandora_data_fusion_msgs::ObstacleInfo>(param, 2);
    }
    else
    {
      ROS_FATAL("[ObjectHandler %d] obstacle_info topic name param not found", __LINE__);
      ROS_BREAK();
    }
  }

  bool ObjectHandler::setQRVisited(int qrId)
  {
    QrPtr visitedQr = qrsToGoList_->removeElementWithId(qrId);
    if (visitedQr.get() == NULL)
      return false;
    if (!qrsVisitedList_->add(visitedQr))
    {
      ROS_ERROR("[/PANDORA_ALERT_HANDLER] qr with id %d has been set visited again.",
          qrId);
    }
    return true;
  }

  void ObjectHandler::handleHoles(const HolePtrVectorPtr& newHoles,
      const tf::Transform& transform)
  {
    keepValidObjects<Hole>(newHoles, transform);
    deleteObjectsOnSoftObstacles<Hole>(newHoles);

    for (int ii = 0; ii < newHoles->size(); ++ii)
    {
      Hole::getList()->add(newHoles->at(ii));
    }
  }

  void ObjectHandler::getQrsInfo(pandora_data_fusion_msgs::WorldModel* worldModelPtr)
  {
    worldModelPtr->qrs.clear();
    worldModelPtr->visitedQrs.clear();

    for (QrList::const_iterator it = qrsToGoList_->begin();
         it != qrsToGoList_->end(); ++it)
    {
      worldModelPtr->qrs.push_back((*it)->getQrInfo());
    }

    for (QrList::const_iterator it = qrsVisitedList_->begin();
         it != qrsVisitedList_->end(); ++it)
    {
      worldModelPtr->visitedQrs.push_back((*it)->getQrInfo());
    }
  }

  void ObjectHandler::updateParams(float sensor_range, float victim_cluster_radius,
      float unreachable_height)
  {
    SENSOR_RANGE = sensor_range;
    VICTIM_CLUSTER_RADIUS = victim_cluster_radius;
    UNREACHABLE_HEIGHT = unreachable_height;
  }

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion
