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

#ifndef PANDORA_ALERT_HANDLER_OBJECTS_HAZMAT_H
#define PANDORA_ALERT_HANDLER_OBJECTS_HAZMAT_H

#include <vector>

#include "pandora_vision_msgs/HazmatAlertVector.h"
#include "pandora_vision_msgs/HazmatAlert.h"
#include "pandora_data_fusion_msgs/HazmatInfo.h"

#include "pandora_alert_handler/objects/object_interface/kalman_object.h"

namespace pandora_data_fusion
{
namespace pandora_alert_handler
{

  /**
    * @class Hazmat
    * @brief Concrete class representing a Hazmat Object. Inherits from Object
    */
  class Hazmat : public KalmanObject<Hazmat>
  {
   public:
    //!< Type Definitions
    typedef pandora_vision_msgs::HazmatAlertVector AlertVector;
    typedef pandora_vision_msgs::HazmatAlert Alert;
    typedef pandora_data_fusion_msgs::HazmatInfo Info;

   public:
    static void setUpObject(const Ptr& ptr, const Alert& msg)
    {
      ptr->setPattern(msg.patternType);
    }

    /**
      * @brief Constructor
      */
    Hazmat();

    virtual bool isSameObject(const ObjectConstPtr& object) const;

    virtual void getVisualization(visualization_msgs::MarkerArray* markers) const;

    virtual void fillGeotiff(const pandora_data_fusion_msgs::
        GetGeotiffResponsePtr& res) const;

    pandora_data_fusion_msgs::HazmatInfo getHazmatInfo() const;

    /**
      * @brief Getter for member pattern_
      * @return int The Hazmat's pattern
      */
    int getPattern() const
    {
      return pattern_;
    }

    /**
      * @brief Setter for member pattern_
      * @return void
      */
    void setPattern(int pattern)
    {
      pattern_ = pattern;
    }

   private:
    //!< The hazmat's pattern
    int pattern_;
  };

  typedef Hazmat::Ptr HazmatPtr;
  typedef Hazmat::ConstPtr HazmatConstPtr;
  typedef Hazmat::PtrVector HazmatPtrVector;
  typedef Hazmat::PtrVectorPtr HazmatPtrVectorPtr;
  typedef Hazmat::List HazmatList;
  typedef Hazmat::ListPtr HazmatListPtr;
  typedef Hazmat::ListConstPtr HazmatListConstPtr;

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

#endif  // PANDORA_ALERT_HANDLER_OBJECTS_HAZMAT_H
