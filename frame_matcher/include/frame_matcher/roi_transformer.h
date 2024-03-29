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

#ifndef FRAME_MATCHER_ROI_TRANSFORMER_H
#define FRAME_MATCHER_ROI_TRANSFORMER_H

#include <string>
#include <vector>
#include <boost/scoped_ptr.hpp>

#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include "frame_matcher/keypoint_transformer.h"
#include "frame_matcher/view_pose_finder.h"

namespace pandora_data_fusion
{
namespace frame_matcher
{
  /**
   * @class RoiTransformer TODO
   */
  class RoiTransformer
  {
   public:
    RoiTransformer(const ros::NodeHandle& nh, const ViewPoseFinderPtr& viewPoseFinderPtr);
    virtual
    ~RoiTransformer();

    void
    transformRegion(const sensor_msgs::Image& imageFrom,
                    const std::vector<cv::Point2f>& roiFrom,
                    const sensor_msgs::Image& imageTo,
                    std::vector<cv::Point2f>* roiToPtr);

   private:
    void
    changeIntoOrthogonalBox(std::vector<cv::Point2f>* roiPtr);

   private:
    KeypointTransformer keypointTransformer_;
  };

  typedef boost::scoped_ptr<RoiTransformer> RoiTransformerPtr;
}  // namespace frame_matcher
}  // namespace pandora_data_fusion

#endif  // FRAME_MATCHER_ROI_TRANSFORMER_H
