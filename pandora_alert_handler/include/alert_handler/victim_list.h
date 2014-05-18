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

#ifndef ALERT_HANDLER_VICTIM_LIST_H
#define ALERT_HANDLER_VICTIM_LIST_H

#include <list>
#include <vector>
#include <map>

#include "pandora_data_fusion_msgs/VictimsMsg.h"
#include "pandora_data_fusion_msgs/VictimInfoMsg.h"
#include "pandora_data_fusion_msgs/VictimVerificationMsg.h"

#include "alert_handler/object_list.h"
#include "alert_handler/utils.h"
#include "alert_handler/victim.h"

namespace pandora_data_fusion
{
namespace pandora_alert_handler
{

class VictimList : public ObjectList<Victim>
{  
 public:
  
  /**
   * @brief Constructor
   * @param counterThreshold [float] Initialization value for counterThreshold
   * @param distanceThreshold [float] Initialization value for distanceThreshold
   * @param approachDistance [float] Initialization value for approachDistance
   */ 
  VictimList();

  /**
   * @brief Checks if the given victim is already in the list
   * @param victim [VictimConstPtr const&] The victim whose existance
   * we need to check
   * @return bool True if victim exists, false otherwise
   */
  bool contains(const VictimConstPtr& victim) const;

  /**
   * @brief Checks if a victim is currently tracked or not
   * @return bool True if a victim is currently tracked, false otherwise
   */
  bool isVictimBeingTracked() const;

  /**
   * @brief Returns the currently tracked victim
   * @return const VictimPtr& A const& to the victim
   */
  const VictimPtr& getCurrentVictim() const; 

  /**
   * @brief Returns a vector containing a VictimInfoMsg for each unvisited victim
   * @param victimsMsg [pandora_data_fusion_msgs::VictimsMsg*] The output vector
   * @return void
   */
  void getVictimsInfo(
    pandora_data_fusion_msgs::VictimsMsg* victimsMsg);

  /**
   * @brief Get the current victim pose as a stamped transform
   * @param stampedTranform [tf::StampedTransform*] The output param
   * @return bool -1 if no victim is tracked
   */
  bool getCurrentVictimTransform(tf::Transform* Transform) const;

  /**
   * @brief Sets the victim index to a specific victim
   * @param victimId [int] current selected victim's unique id
   * @return bool true, if selected successfully or -1, false, if not found.
   */
  bool setCurrentVictim(int victimId);

  /**
   * @brief Deletes VictimPtr with the corresponding victimId
   * @param victimId [int] id that will be used to search for the victim
   * @return bool true, if deleted, false, if not found.
   */
  bool deleteVictim(int victimId);

  /**
   * @brief Validates victim with victimId.
   * @param victimId [int] current victim's unique id
   * @param victimValid [bool] If current victim is valid or not
   * @return VictimPtr pointer to current victim
   */
  VictimPtr validateVictim(int victimId, bool objectValid);

  /**
   * @brief Adds the given victim to the list as is, no checks and updates
   * @param victim [VictimPtr const&] The victim to be added
   * @return void
   */
  void addUnchanged(const VictimPtr& victim);

  /**
   * @brief Checks if the approach pose of current victim has significally changed
   * since start of tracking
   * @return bool True if approach pose changed a significally, false otherwise
   */
  bool currentVictimUpdated(); 

  /**
   * @override
   */
  void clear();

  /**
   * @overload
   */
  void setParams(float approachDistance, float victimUpdate);

 protected:

  /**
   * @override
   */
  void updateObjects(const ConstPtr& victim,
    const IteratorList& iteratorList);
  
 protected:
 
  //!< An iterator pointing to the currently tracked victim.
  iterator currentVictimIt_;
  //!< The pose of the currently tracked victim when tracking was last updated.
  geometry_msgs::Pose currentApproachPose_;
  //!< True if the victims were requested and given, false otherwise.
  bool currentVictimDied_;
  
  //!< The distance of the approach pose from the wall.
  float APPROACH_DIST;
  //!< The approach pose distance threshold for informing fsm of change.
  float VICTIM_UPDATE;

 private:

  friend class VictimListTest;
 
};

typedef boost::shared_ptr<VictimList> VictimListPtr;
typedef boost::shared_ptr<VictimList const> VictimListConstPtr;

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

#endif  // ALERT_HANDLER_VICTIM_LIST_H
