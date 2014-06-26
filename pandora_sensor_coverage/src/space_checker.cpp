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
#include <vector>

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

      //coveredSpace_.reset( new nav_msgs::OccupancyGrid );
    }

    void SpaceChecker::findCoverage(const tf::StampedTransform& sensorTransform,
        const tf::StampedTransform& baseTransform)
    {
      alignCoverageWithMap();

      CoverageChecker::findCoverage(sensorTransform);

      const float resolution = map2d_->info.resolution;
      float minZ = baseTransform.getOrigin()[2];
      float currX = position_.x();
      float currY = position_.y();
      float fov = (SENSOR_HFOV / 180.0) * PI;
      octomap::point3d cell;

      for (float angle = -fov/2; angle < fov/2; angle += DEGREE)
      {
        cell.x() = resolution * cos(yaw_ + angle) + currX;
        cell.y() = resolution * sin(yaw_ + angle) + currY;

        while (CELL(cell.x(), cell.y(), map2d_) 
            < static_cast<int8_t>(OCCUPIED_CELL_THRES * 100)
            && Utils::distanceBetweenPoints2D(octomap::pointOctomapToMsg(position_),
              octomap::pointOctomapToMsg(cell)) < SENSOR_RANGE)
        {
          //signed char covered = static_cast<signed char>(floor(cellCoverage(cell, minZ) * 100));
          signed char covered = 254;
          if (covered > CELL(cell.x(), cell.y(), (&coveredSpace_))) 
          {
            CELL(cell.x(), cell.y(), (&coveredSpace_)) = covered;
          }
          coverageDilation(1, COORDS(cell.x(), cell.y(), (&coveredSpace_)));
          cell.x() += resolution * cos(yaw_ + angle);
          cell.y() += resolution * sin(yaw_ + angle);
        }
      }

      unsigned int cellsCovered = 0;
      for (int ii = 0; ii < coveredSpace_.info.width; ++ii)
      {
        for (int jj = 0; jj < coveredSpace_.info.height; ++jj)
        {
          if (map2d_->data[ii + jj * map2d_->info.width] >= static_cast<int8_t>(OCCUPIED_CELL_THRES * 100))
          {
            coveredSpace_.data[ii + jj * coveredSpace_.info.width] = 0;
          }
          if (coveredSpace_.data[ii + jj * coveredSpace_.info.width] != 0)
          {
            cellsCovered++;
          }
        }
      }
      totalAreaCovered_ = cellsCovered * resolution * resolution;
      ROS_INFO_THROTTLE(20,
          "[SENSOR_COVERAGE_SPACE_CHECKER %d] Total area covered is %f m^2.",
          __LINE__, totalAreaCovered_);
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
    float SpaceChecker::cellCoverage(const octomap::point3d& cell, float minHeight)
    {
      octomap::point3d end(cell.x(), cell.y(), minHeight);
      //  begin can be later implemented having z = minHeight + MAX_HEIGHT.
      octomap::point3d begin(cell.x(), cell.y(), MAX_HEIGHT);
      octomap::KeyRay keyRay;
      if (!map3d_->computeRayKeys(begin, end, keyRay))
      {
        ROS_ERROR("Compute ray went out of range!");
      }
      bool coversSpace = false, occupied = false;
      float coveredSpace = 0, startZ = begin.z(), unoccupiedSpace = 0;
      for (octomap::KeyRay::iterator it = keyRay.begin();
          it != keyRay.end(); ++it)
      {
        octomap::OcTreeNode* node = map3d_->search(*it);
        if (!node)
        {
          continue;
        }
        if (occupied)
        {
          if (node->getOccupancy() == 0)
          {
            //  In 3d navigation planning this must be changed.
            occupied = false;
            coversSpace = false;
            break;
          }
          else if (node->getOccupancy() <= map3d_->getOccupancyThres())
          {
            occupied = false;
            coversSpace = true;
            startZ = map3d_->keyToCoord(*it).z();
          }
        }
        else if (coversSpace)
        {
          if (node->getOccupancy() == 0 || it == --keyRay.end())
          {
            coversSpace = false;
            octomap::point3d coord = map3d_->keyToCoord(*it);
            coveredSpace += startZ - coord.z();
            unoccupiedSpace += startZ - coord.z();
            startZ = coord.z();
          }
          else if (node->getOccupancy() > map3d_->getOccupancyThres())
          {
            occupied = true;
            coversSpace = false;
            octomap::point3d coord = map3d_->keyToCoord(*it);
            coveredSpace += startZ - coord.z();
            unoccupiedSpace += startZ - coord.z();
          }
        }
        else
        {
          if (node->getOccupancy() > map3d_->getOccupancyThres())
          {
            occupied = true;
            unoccupiedSpace += startZ - map3d_->keyToCoord(*it).z();
          }
          else if (node->getOccupancy() > 0)
          {
            coversSpace = true;
            octomap::point3d coord = map3d_->keyToCoord(*it);
            unoccupiedSpace += startZ - coord.z();
            startZ = coord.z();
          }
        }
      }
      return coveredSpace / unoccupiedSpace;
    }

    void SpaceChecker::alignCoverageWithMap()
    {
      int oldSize = coveredSpace_.data.size();
      int newSize = map2d_->data.size();
      int8_t oldCoverage[oldSize];
      nav_msgs::MapMetaData oldMetaData;
      if (oldSize != 0 && oldSize != newSize)
      {
        // Copy old coverage map meta data.
        oldMetaData = coveredSpace_.info;
        // Copy old coverage map.
        for (unsigned int ii = 0; ii < oldSize; ++ii)
        {
          oldCoverage[ii] = coveredSpace_.data[ii];
        }
      }
      // Reset coveredSpace_ and copy map2D_'s metadata.
      coveredSpace_.header = map2d_->header;
      coveredSpace_.info = map2d_->info;
      if (oldSize != newSize)
      {
        ROS_WARN("[SENSOR_COVERAGE_SPACE_CHECKER %d] Resizing space coverage...", __LINE__);
        coveredSpace_.data.resize(newSize, 0);
        ROS_ASSERT(newSize == coveredSpace_.data.size());

        if (oldSize != 0)
        {
          double yawDiff = tf::getYaw(coveredSpace_.info.origin.orientation) -
            tf::getYaw(oldMetaData.origin.orientation);
          double xDiff = coveredSpace_.info.origin.position.x -
            oldMetaData.origin.position.x;
          double yDiff = coveredSpace_.info.origin.position.y -
            oldMetaData.origin.position.y;

          double x = 0, y = 0, xn = 0, yn = 0;
          for (unsigned int ii = 0; ii < oldMetaData.width; ++ii)
          {
            for (unsigned int jj = 0; jj < oldMetaData.height; ++jj)
            {
              x = ii * oldMetaData.resolution;
              y = jj * oldMetaData.resolution;
              xn = cos(yawDiff) * x - sin(yawDiff) * y - xDiff;
              yn = sin(yawDiff) * x + cos(yawDiff) * y - yDiff;
              int coords = int(floor((xn + yn * coveredSpace_.info.width) 
                    / coveredSpace_.info.resolution));
              coveredSpace_.data[coords] = oldCoverage[ii + jj * oldMetaData.width];
              coverageDilation(1, COORDS(xn, yn, (&coveredSpace_)));
            }
          }
        }
      }
    }

    void SpaceChecker::coverageDilation(int steps, int coords)
    {
      if (steps == 0)
        return;

      signed char cell = coveredSpace_.data[coords];

      if (cell != 0) // That's foreground
      {
        // Check for all adjacent
        if (coveredSpace_.data[coords + coveredSpace_.info.width + 1] == 0)
        {
          coveredSpace_.data[coords + coveredSpace_.info.width + 1] = cell;
          coverageDilation(steps - 1, coords + coveredSpace_.info.width + 1);
        }
        if (coveredSpace_.data[coords + coveredSpace_.info.width] == 0)
        {
          coveredSpace_.data[coords + coveredSpace_.info.width] = cell;
        }
        if (coveredSpace_.data[coords + coveredSpace_.info.width - 1] == 0)
        {
          coveredSpace_.data[coords + coveredSpace_.info.width - 1] = cell;
          coverageDilation(steps - 1, coords + coveredSpace_.info.width - 1);
        }
        if (coveredSpace_.data[coords + 1] == 0)
        {
          coveredSpace_.data[coords + 1] = cell;
        }
        if (coveredSpace_.data[coords - 1] == 0)
        {
          coveredSpace_.data[coords - 1] = cell;
        }
        if (coveredSpace_.data[coords - coveredSpace_.info.width + 1] == 0)
        {
          coveredSpace_.data[coords - coveredSpace_.info.width + 1] = cell;
          coverageDilation(steps - 1, coords - coveredSpace_.info.width + 1);
        }
        if (coveredSpace_.data[coords - coveredSpace_.info.width] == 0)
        {
          coveredSpace_.data[coords - coveredSpace_.info.width] = cell;
        }
        if (coveredSpace_.data[coords - coveredSpace_.info.width - 1] == 0)
        {
          coveredSpace_.data[coords - coveredSpace_.info.width - 1] = cell;
          coverageDilation(steps - 1, coords - coveredSpace_.info.width - 1);
        }
      }
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

