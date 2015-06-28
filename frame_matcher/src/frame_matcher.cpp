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

#include "state_manager_msgs/RobotModeMsg.h"

#include "frame_matcher/frame_matcher.h"

namespace pandora_data_fusion
{
namespace frame_matcher
{

  FrameMatcher::
  FrameMatcher(const std::string& ns) :
    sensor_processor::Handler(ns),
    PostProcessorLoader_("sensor_processor", "sensor_processor::AbstractProcessor")
  {
    ros::NodeHandle private_nh("~");
    name_ = private_nh.getNamespace();
    // Get PostProcessor's implementation name
    if (!private_nh.getParam("post_processor", postProcessorName_))
    {
      ROS_FATAL("[FRAME_MATCHER %d] post processor class name param not found "
                "for %s", __LINE__, name_.c_str());
      ROS_BREAK();
    }
    // Get Active States
    XmlRpc::XmlRpcValue active_states;
    activeStates_.clear();
    if (!private_nh.getParam("robot_states_on", active_states))
    {
      ROS_FATAL("[FRAME_MATCHER %d] robot states, at which node is on, were not"
                " found for %s", __LINE__, name_.c_str());
    }
    ROS_ASSERT(active_states.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for (int ii = 0; ii < active_states.size(); ++ii) {
      ROS_ASSERT(active_states[ii].getType() == XmlRpc::XmlRpcValue::TypeInt);
      activeStates_.push_back(static_cast<int>(active_states[ii]));
    }
  }

  FrameMatcher::
  ~FrameMatcher() {}

  /**
   * @details TODO
   */
  void
  FrameMatcher::
  startTransition(int newState)
  {
    currentState_ = newState;

    bool previouslyOff = true;
    bool currentlyOn = false;

    for (int ii = 0; ii < activeStates_.size(); ii++) {
      previouslyOff = (previouslyOff && previousState_ != activeStates_[ii]);
      currentlyOn = (currentlyOn || currentState_ == activeStates_[ii]);
    }

    if (previouslyOff && currentlyOn)
    {
      preProcPtr_.reset( new MatcherPreProcessor("~preprocessor", this) );
      processorPtr_.reset( new MatcherProcessor("~", this) );
      // plugin post processor
      loadPostProcessor(postProcessorName_);
    }
    else if (!previouslyOff && !currentlyOn)
    {
      preProcPtr_.reset();
      processorPtr_.reset();
      postProcPtr_.reset();
    }

    if (currentState_ == state_manager_msgs::RobotModeMsg::MODE_TERMINATING)
    {
      preProcPtr_.reset();
      processorPtr_.reset();
      postProcPtr_.reset();

      ros::shutdown();
      return;
    }
    previousState_ = currentState_;
    transitionComplete(currentState_);
  }

  /**
   * @details TODO
   */
  void
  FrameMatcher::
  completeTransition()
  {
  }

  /**
   * @details TODO
   */
  void
  FrameMatcher::
  loadPostProcessor(const std::string& name)
  {
    try
    {
      postProcPtr_ = postProcessorLoader_.createInstance(name);
      postProcPtr_->initialize("~postprocessor", this);
    }
    catch (const pluginlib::PluginlibException& ex)
    {
      ROS_FATAL("[FRAME_MATCHER %d] Failed to create the %s processor, are you sure it is properly"
                " registered and that the containing library is built? "
                "Exception: %s", __LINE__, name.c_str(), ex.what());
      ROS_BREAK();
    }
  }

}  // namespace frame_matcher
}  // namespace pandora_data_fusion
