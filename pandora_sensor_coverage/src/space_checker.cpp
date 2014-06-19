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
    double SpaceChecker::OCCUPIED_CELL_THRES = 0.5;

    void SpaceChecker::findCoverage(const tf::StampedTransform& sensorTransform,
        const tf::StampedTransform& baseTransform)
    {
      alignCoverageWithMap();

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
          CELL(cell.x(), cell.y(), (&coveredSpace_)) = covered;
          cell.x() += resolution * cos(yaw_ + angle);
          cell.y() += resolution * sin(yaw_ + angle);
        }
      }

      for (int ii = 0; ii < coveredSpace_.info.width; ++ii)
      {
        for (int jj = 0; jj < coveredSpace_.info.height; ++jj)
        {
          if (map2D_->data[ii + jj * map2D_->info.width] >= OCCUPIED_CELL_THRES * 100)
          {
            coveredSpace_.data[ii + jj * coveredSpace_.info.width] = 0;
          }
        }
      }
    }

    /**
     * @details Calculates space covered above a cell in 2D map as a percentage of
     * covered space over unoccupied space according to current 3D map. Vertical
     * raytracing goes top to bottom and stops either when the node examined is
     * in the same height as base footprint or when an occupied node is found and
     * the space below it is uncovered. This method is assuming that there is
     * no way that the robot's base footprint can have different z for the same (x, y),
     * but it assures that coverage as a percentage is true at all times and it is not
     * dependent of current z.
     */
    float SpaceChecker::cellCoverage(const octomath::Vector3& cell, float minHeight)
    {
      octomath::Vector3 end(cell.x(), cell.y(), minHeight);
      //  begin can be later implemented having z = minHeight + MAX_HEIGHT.
      octomath::Vector3 begin(cell.x(), cell.y(), MAX_HEIGHT);
      std::vector<octomath::Vector3> pointVector;
      map3D_->computeRay(begin, end, pointVector);
      bool coversSpace = false, occupied = false;
      float coveredSpace = 0, startZ = pointVector[0].z(), unoccupiedSpace = 0;
      for (unsigned int ii = 0;
          ii < pointVector.size(); ++ii)
      {
        octomap::OcTreeNode* node = map3D_->search(pointVector[ii]);
        if (occupied)
        {
          if (node->getOccupancy() == 0)
          {
            //  In 3D navigation planning this must be changed.
            occupied = false;
            coversSpace = false;
            break;
          }
          else if (node->getOccupancy() <= map3D_->getOccupancyThres())
          {
            occupied = false;
            coversSpace = true;
            startZ = pointVector[ii].z();
          }
        }
        else if (coversSpace)
        {
          if (node->getOccupancy() == 0 || ii == pointVector.size() - 1)
          {
            coversSpace = false;
            coveredSpace += startZ - pointVector[ii].z();
            unoccupiedSpace += startZ - pointVector[ii].z();
            startZ = pointVector[ii].z();
          }
          else if (node->getOccupancy() > map3D_->getOccupancyThres())
          {
            occupied = true;
            coversSpace = false;
            coveredSpace += startZ - pointVector[ii].z();
            unoccupiedSpace += startZ - pointVector[ii].z();
          }
        }
        else
        {
          if (node->getOccupancy() > map3D_->getOccupancyThres())
          {
            occupied = true;
            unoccupiedSpace += startZ - pointVector[ii].z();
          }
          else if (node->getOccupancy() > 0)
          {
            coversSpace = true;
            unoccupiedSpace += startZ - pointVector[ii].z();
            startZ = pointVector[ii].z();
          }
        }
      }
      return coveredSpace / unoccupiedSpace; 
    }

    void SpaceChecker::alignCoverageWithMap()
    {
      nav_msgs::OccupancyGrid newCoverage;
      newCoverage.header = map2D_->header;
      newCoverage.info = map2D_->info;
      newCoverage.data.resize(newCoverage.info.width * newCoverage.info.height);

      double yawDiff = tf::getYaw(newCoverage.info.origin.orientation) -
        tf::getYaw(coveredSpace_.info.origin.orientation);
      double xDiff = newCoverage.info.origin.position.x - 
        coveredSpace_.info.origin.position.x;
      double yDiff = newCoverage.info.origin.position.y - 
        coveredSpace_.info.origin.position.y;
      
      unsigned int ii = 0, jj = 0, iin = 0, jjn = 0;
      double x = 0, y = 0, xn = 0, yn = 0;
      for (ii = 0; ii < coveredSpace_.info.width; ++ii)
      {
        for (jj = 0; jj < coveredSpace_.info.height; ++jj)
        {
          x = coveredSpace_.info.origin.position.x + ii * coveredSpace_.info.resolution;
          y = coveredSpace_.info.origin.position.y + jj * coveredSpace_.info.resolution;
          xn = cos(yawDiff) * x - sin(yawDiff) * y + xDiff;
          yn = sin(yawDiff) * x + cos(yawDiff) * y + yDiff;
          CELL(xn, yn, (&newCoverage)) = coveredSpace_.data[
            ii + jj * coveredSpace_.info.width];
        }
      }

      coveredSpace_ = newCoverage;
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

