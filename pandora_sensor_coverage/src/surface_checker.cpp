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

#include "sensor_coverage/surface_checker.h"

namespace pandora_data_fusion
{
  namespace pandora_sensor_coverage
  {

    SurfaceChecker::SurfaceChecker(const NodeHandlePtr& nh, const std::string& frameName)
      : CoverageChecker(nh, frameName)
    {
      std::string topic;

      if (nh_->getParam("published_topic_names/surface_"+frameName_, topic))
      {
        coveragePublisher_ = nh_->advertise<octomap_msgs::Octomap>(topic, 1);
      }
      else
      {
        ROS_FATAL("%s topic name param not found", frameName_.c_str());
        ROS_BREAK();
      }

      getParameters();
    }

    void SurfaceChecker::findCoverage(const tf::StampedTransform& transform)
    {
      //  Initialize surface coverage map, if uninitialized.
      if (coveredSurface_.get() == NULL)
        coveredSurface_.reset( new octomap::ColorOcTree(map3d_->getResolution()) );
      CoverageChecker::findCoverage(transform);
      double yaw_curr = 0, pitch_curr = 0;
      double h_fov = (SENSOR_HFOV / 180.0) * PI;
      double v_fov = (SENSOR_VFOV / 180.0) * PI;
      octomap::point3d pointOnWall;
      for (double h_angle = -h_fov / 2; h_angle < h_fov / 2; h_angle += DEGREE)
      {
        for (double v_angle = -v_fov / 2; v_angle < v_fov / 2; v_angle += DEGREE)
        {
          yaw_curr = yaw_ + h_angle;
          pitch_curr = pitch_ + v_angle;
          octomap::point3d direction(1, 0, 0);
          if (map3d_->castRay(position_,
                direction.rotate_IP(roll_, pitch_curr, yaw_curr),
                pointOnWall,
                true,
                SENSOR_RANGE))
          {
            if (coveredSurface_->insertRay(position_,
                pointOnWall,
                SENSOR_RANGE,
                true))
            {
              octomap::ColorOcTreeNode* node = coveredSurface_->search(pointOnWall);
              if (node != NULL)
              {
                unsigned char coverage = 0;
                bool toSet = true;
                if (node->isColorSet())
                {
                  coverage = findPointCoverage(pointOnWall, direction);
                  if (node->getColor().r >= coverage)
                    toSet = false;
                }
                if (toSet)
                  node->setColor(coverage, 0, 0);
              }
            }
          }
        }
      }
      coveredSurface_->updateInnerOccupancy();
    }

    unsigned char SurfaceChecker::findPointCoverage(const octomap::point3d& pointOnWall,
        const octomap::point3d& direction)
    {
      return 255;
    }

    void SurfaceChecker::publishCoverage()
    {
      octomap_msgs::Octomap msg;
      if (octomap_msgs::fullMapToMsg(*coveredSurface_, msg))
        coveragePublisher_.publish(msg);
    }

}  // namespace pandora_sensor_coverage
}  // namespace pandora_data_fusion

