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
 *   Christos Zalidis <zalidis@gmail.com>
 *   Triantafyllos Afouras <afourast@gmail.com>
 *********************************************************************/

#ifndef ALERT_HANDLER_VICTIM_HANDLER_H
#define ALERT_HANDLER_VICTIM_HANDLER_H

#include <map>
#include <string>
#include <vector>
#include <boost/utility.hpp>

#include <ros/ros.h>

#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>

#include "pandora_data_fusion_msgs/VictimInfoMsg.h"
#include "pandora_data_fusion_msgs/VictimsMsg.h"
#include "pandora_data_fusion_msgs/DatafusionGeotiffSrv.h"

#include "alert_handler/victim.h"
#include "alert_handler/victim_clusterer.h"
#include "alert_handler/victim_list.h"
#include "alert_handler/defines.h"

namespace pandora_data_fusion
{
  namespace pandora_alert_handler
  {

    /**
     * @class VictimHandler
     * @brief Controller that keeps track of victims 
     */ 
    class VictimHandler : private boost::noncopyable
    {
      public:

        /**
         * @brief Constructor
         */
        VictimHandler(VictimListPtr victimsToGoList,
            VictimListPtr victimsVisitedList);

        /**
         * @brief Updates the victim lists with the existing objects
         * @return void
         */
        void notify();

        /**
         * @brief Check if current alert can be associated with a victim.
         * Verify if possible this victim.
         * @return void
         */
        void inspect();

        /**
         * @brief Updates the victim handler's parameters
         * @param clusterRadius [float] The new cluster radius
         * @param sameVictimRadius [float] The new same victim radius
         * @return void
         */
        void updateParams(float clusterRadius, float sameVictimRadius);

        /**
         * @brief Clears unvisited victims list and selected victim index (if any)
         * @return void
         */
        void flush();

        /**
         * @brief Sets the current victim.
         * @param index [int] selected victim's unique id.
         * @return bool true, if selected successfully, false, if not found.
         */
        bool selectCurrentVictim(int victimId);

        /**
         * @brief Deletes VictimPtr from VictimsToGo list with victimId.
         * @param victimId [int] victim's id
         * @return bool true if deleted successfully, false if not found
         */
        bool deleteVictim(int victimId);

        /**
         * @brief Validates current victim.
         * @param victimId [int] Id of victim to be validated.
         * @param victimValid [bool] True for positive and false for 
         * negative validation
         * @return bool true, if succeded, false, if not found
         */
        bool validateVictim(int victimId, bool victimValid);

        /**
         * @brief Fills victimsMsg with information about victims to go.
         * (PoseStamped, probability, sensors)
         * @param victimsMsg [pandora_data_fusion_msgs::VictimsMsg*] msg to be filled
         * @return void
         */
        void getVictimsInfo(pandora_data_fusion_msgs::VictimsMsg* victimsMsg);

        /**
         * @brief Get the current victim pose as a stamped transform
         * @param stampedTranform [tf::StampedTransform*] The output param
         * @return bool -1 if no victim is tracked
         */
        bool getCurrentVictimTransform(tf::StampedTransform* stampedTranform);

        /**
         * @brief Get Poses Stamped of all victims in victimsToGo, victimsVisited and
         * their respective approachPoints
         * @param victimsToGo [PoseStampedVector*] vector with victims' poses
         * @param victimsVisited [PoseStampedVector*] vector with victims' poses
         * @param approachPoints [PoseStampedVector*] vector with approach points.
         * @return void
         */
        void getVictimsPosesStamped(PoseStampedVector* victimsToGo, 
            PoseStampedVector* victimsVisited);

        /**
         * @brief Fill in the geotiff info with the victims details 
         * @param res 
         * [pandora_data_fusion_msgs::DatafusionGeotiffSrv::Response*] 
         * The output service response param
         * @return void
         */
        void fillGeotiff(
            pandora_data_fusion_msgs::DatafusionGeotiffSrv::Response* res);

        /**
         * @brief Get the victims visualization
         * @param victimsVisitedMarkers [visualization_msgs::MarkerArray*] 
         * The output param for the visited victims 
         * @param victimsToGoMarkers [visualization_msgs::MarkerArray*] 
         * The output param for the unvisited victims 
         * @return void
         */
        void getVisualization(visualization_msgs::MarkerArray* victimsVisitedMarkers, 
            visualization_msgs::MarkerArray* victimsToGoMarkers);

      private:

        /**
         * @brief Returns all the legit Objects from all the Object lists 
         * concatenated into a single vector
         * @return ObjectConstPtrVector A vector contaiing all the legit Objects
         */
        ObjectConstPtrVectorPtr getAllLegitObjects();

      private:

        //!< publisher for valid victims counter
        ros::Publisher validVictimsPublisher_;

        VictimClustererPtr clusterer_;

        //!< The unvisited victims list  
        VictimListPtr victimsToGoList_;
        //!< The visited victims list  
        VictimListPtr victimsVisitedList_;

        //!< counts verified victims that were validated by user
        int validVictimsCounter_;

        //!< Radius within which all legit objects are associated with a victim.
        float CLUSTER_RADIUS; 
    };

    typedef boost::scoped_ptr<VictimHandler> VictimHandlerPtr;

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

#endif  // ALERT_HANDLER_VICTIM_HANDLER_H