/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, P.A.N.D.O.R.A. Team.
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
#include <vector>

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "state_manager_msgs/RobotModeMsg.h"
#include "sensor_processor/dynamic_handler.h"

#include "frame_matcher/matcher_processor.h"
#include "frame_matcher/enhanced_image_preprocessor.h"
#include "frame_matcher/enhanced_image_postprocessor.h"
#include "frame_matcher/frame_matcher.h"

PLUGINLIB_EXPORT_CLASS(pandora_data_fusion::frame_matcher::FrameMatcher,
    nodelet::Nodelet)

namespace pandora_data_fusion
{
namespace frame_matcher
{

  FrameMatcher::
  FrameMatcher() :
    sensor_processor::DynamicHandler() {}

  void
  FrameMatcher::
  onInit()
  {
    sensor_processor::DynamicHandler::onInit();

    private_nh_ = this->getPrivateNodeHandle();
    name_ = this->getName();

    if (!private_nh_.getParam("preprocessor", preprocessor_type_))
    {
      ROS_FATAL("[%s] Cound not find matcher preprocessor", name_.c_str());
      ROS_BREAK();
    }

    if (!private_nh_.getParam("postprocessor", preprocessor_type_))
    {
      ROS_FATAL("[%s] Cound not find matcher postprocessor", name_.c_str());
      ROS_BREAK();
    }
  }

  void
  FrameMatcher::
  startTransition(int newState)
  {
    this->previousState_ = this->currentState_;
    this->currentState_ = newState;

    bool previouslyOff = true;
    bool currentlyOn = false;

    for (int ii = 0; ii < activeStates_.size(); ii++) {
      previouslyOff = (previouslyOff && this->previousState_ != ROBOT_STATES(activeStates_[ii]));
      currentlyOn = (currentlyOn || this->currentState_ == ROBOT_STATES(activeStates_[ii]));
    }

    if (!previouslyOff && !currentlyOn)
    {
      unloadPreProcessor();
      unloadProcessor();
      unloadPostProcessor();
    }
    else if (previouslyOff && currentlyOn)
    {
      loadPreProcessor<EnhancedImagePreProcessor>("~preprocessor");
      // loadPreProcessor<EnhancedImagePreProcessor>("~");
      loadProcessor<MatcherProcessor>("~matcher");
      loadPostProcessor<EnhancedImagePostProcessor>("~postprocessor");
      // loadPostProcessor("~postprocessor", postprocessor_type_);
    }

    if (this->currentState_ == state_manager_msgs::RobotModeMsg::MODE_TERMINATING)
    {
      unloadPreProcessor();
      unloadProcessor();
      unloadPostProcessor();

      ROS_INFO("[%s] Terminating", name_.c_str());
      ros::shutdown();
      return;
    }

    transitionComplete(this->currentState_);
  }
}  // namespace frame_matcher
}  // namespace pandora_data_fusion
