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

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/Image.h>

#include "sensor_processor/handler.h"
#include "sensor_processor/processor_error.h"

#include "frame_matcher/points_on_frame.h"
#include "frame_matcher/matcher_processor.h"

namespace pandora_data_fusion
{
namespace frame_matcher
{

  /**
   * @details TODO
   */
  MatcherProcessor::
  MatcherProcessor(const std::string ns, sensor_processor::Handler* handler)
  {
    initialize(ns, handler);

    nh_ = *(this->accessPublicNh());
    processor_nh_ = *(this->accessProcessorNh());
    processorName_ = this->getName();

    if (!processor_nh_.getParam("image_target_topic", imageToTopicName_))
    {
      ROS_FATAL("[%s] Cound not find image target topic name", processorName_.c_str());
      ROS_BREAK();
    }

    if (!processor_nh_.getParam("map_topic", mapTopicName_))
    {
      ROS_FATAL("[%s] Cound not fing map topic name", processorName_.c_str());
      ROS_BREAK();
    }

    std::string map_type;
    if (!nh_.getParam("map_type", map_type))
    {
      ROS_FATAL("[%s] Cound not fing map type", processorName_.c_str());
      ROS_BREAK();
    }

    imageToSubscriber_ = nh_.subscribe(imageToTopicName_, 1,
        &MatcherProcessor::updateImageTo, this);
    mapSubscriber_ = nh_.subscribe(mapTopicName_, 1,
        &MatcherProcessor::updateMap, this);

    viewPoseFinderPtr_.reset( new ViewPoseFinder(map_type) );
    roiTransformer_.reset( new RoiTransformer(nh_, viewPoseFinderPtr_) );
  }

  MatcherProcessor::
  ~MatcherProcessor() {}

  /**
   * @details TODO
   */
  void
  MatcherProcessor::
  preProcess(const PointsOnFrameConstPtr& input, const PointsOnFramePtr& output)
  {
    if (mapConstPtr_.get() == NULL)
      throw sensor_processor::processor_error("Map is not initialized yet!");

    // Set output correctly from input
    output->header = input->header;
    output->rgbImage = input->rgbImage;
    output->points.clear();
    // Give Roi rgb sensor image and points
    roiTransformer_->transformRegion(input->rgbImage, input->points,
        *imageToConstPtr_, &output->points);
    return true;
  }

  void
  MatcherProcessor::
  updateMap(const nav_msgs::OccupancyGridConstPtr& msg)
  {
    mapConstPtr_ = msg;
    viewPoseFinderPtr_->updateMap(msg);
  }

  void
  MatcherProcessor::
  updateImageTo(const sensor_msgs::ImageConstPtr& msg)
  {
    imageToConstPtr_ = msg;
  }

}  // namespace frame_matcher
}  // namespace pandora_data_fusion
