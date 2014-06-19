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

#include "sensor_coverage/space_checker.h"

namespace pandora_data_fusion
{
  namespace pandora_sensor_coverage
  {

    SpaceChecker::SpaceChecker(const NodeHandlePtr& nh, const std::string& frameName)
      : CoverageChecker(nh, frameName)
    {
      std::string topic;

      if (nh_->getParam("published_topic_names/space_"+frameName_, topic))
      {
        coveragePublisher_ = nh_->advertise<nav_msgs::OccupancyGrid>(topic, 1);
      }
      else
      {
        ROS_FATAL("%s topic name param not found", frameName_.c_str());
        ROS_BREAK();
      }

      getParameters();
    }

    boost::shared_ptr<nav_msgs::OccupancyGrid> SpaceChecker::map2D_;
    float SpaceChecker::OCCUPIED_CELL_THRES = 0.5;

    void SpaceChecker::findCoverage(const tf::StampedTransform& sensorTransform,
        const tf::StampedTransform& baseTransform)
    {
      CoverageChecker::findCoverage(sensorTransform);

      const float resolution = map2D_->info.resolution;
      float step = 5 * resolution;
      float minZ = baseTransform.getOrigin()[2];
      float currX = position_.x;
      float currY = position_.y;
      float fov = (SENSOR_HFOV / 180.0) * PI;
      octomath::Vector3 cell;

      for (float angle = -fov/2; angle < fov/2; angle += PI / 180.0)
      {
        cell.x() = step * cos(yaw_ + angle) + currX;
        cell.y() = step * sin(yaw_ + angle) + currY;

        while (CELL(cell.x(), cell.y(), map2D_) < OCCUPIED_CELL_THRES * 100
            && Utils::distanceBetweenPoints2D(
              position_, octomap::pointOctomapToMsg(cell)) < SENSOR_RANGE)
        {
          int covered = ceil(cellCoverage(cell, minZ) * 100);
          if (covered > CELL(cell.x(), cell.y(), (&coveredSpace_)))
          {
            CELL(cell.x(), cell.y(), (&coveredSpace_)) = covered;
          }
          cell.x() += resolution * cos(yaw_ + angle);
          cell.y() += resolution * sin(yaw_ + angle);
        }
      }

      for (int ii = 0; ii < map2D_->info.width; ++ii)
      {
        for (int jj = 0; jj < map2D_->info.height; ++jj)
        {
          if (map2D_->data[ii + jj * map2D_->info.width] >= OCCUPIED_CELL_THRES * 100)
          {
            coveredSpace_.data[ii + jj * coveredSpace_.info.width] = 0;
          }
        }
      }
    }

    float SpaceChecker::cellCoverage(const octomath::Vector3& cell, float minHeight)
    {
      bool coversSpace = false, occupied = false;
      float totalCoverage = 0, startZ = 0, occupiedHeight = 0;
      octomath::Vector3 begin(cell.x(), cell.y(), minHeight);
      //  end can be later implemented having z = minHeight + MAX_HEIGHT
      octomath::Vector3 end(cell.x(), cell.y(), MAX_HEIGHT);
      std::vector<octomath::Vector3> pointVector;
      map3D_->computeRay(begin, end, pointVector);
      for (unsigned int ii = 0;
          ii < pointVector.size(); ++ii)
      {
        octomap::OcTreeNode* node = map3D_->search(pointVector[ii]);
        if (!coversSpace &&
            node->getOccupancy() <= map3D_->getOccupancyThres() &&
            node->getOccupancy() > 0)
        {
          coversSpace = true;
          startZ = pointVector[ii].z();
        }
        if (coversSpace)
        {
          if (node->getOccupancy() == 0 || ii == pointVector.size() - 1)
          {
            coversSpace = false;
            if (occupied)
            {
              occupied = false;
              occupiedHeight += pointVector[ii].z() - startZ;
            }
            else
            {
              totalCoverage += pointVector[ii].z() - startZ;
            }
          }
          else if (node->getOccupancy() > map3D_->getOccupancyThres())
          {
            totalCoverage += pointVector[ii].z() - startZ;
            occupied = true;
            startZ = pointVector[ii].z();
          }
          else if (occupied)
          {
            occupied = false;
            occupiedHeight += pointVector[ii].z() - startZ;
            startZ = pointVector[ii].z();
          }
        }
      }
      float totalHeight = pointVector[pointVector.size() - 1].z() - pointVector[0].z();
      return totalCoverage / (totalHeight - occupiedHeight); 
    }

    void SpaceChecker::publishCoverage()
    {
      coveragePublisher_.publish(coveredSpace_);
    }

    void SpaceChecker::getParameters()
    {
      CoverageChecker::getParameters();
      if (!nh_->getParam("max_height/"+frameName_, MAX_HEIGHT))
      {
        ROS_FATAL("%s maximum height of interest param not found", frameName_.c_str());
        ROS_BREAK();
      }
    }

}  // namespace pandora_sensor_coverage
}  // namespace pandora_data_fusion

