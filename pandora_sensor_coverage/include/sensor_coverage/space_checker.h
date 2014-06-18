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

#ifndef SENSOR_COVERAGE_SPACE_CHECKER_H
#define SENSOR_COVERAGE_SPACE_CHECKER_H

#include <string>
#include <boost/shared_ptr.hpp>

#include "nav_msgs/OccupancyGrid.h"

#include "sensor_coverage/coverage_checker.h"

namespace pandora_data_fusion
{
  namespace pandora_sensor_coverage
  {

    /**
     * @brief class that correspond to a tracked sensor.
     * Contains methods that draw coverage into current map.
     */
    class SpaceChecker : public CoverageChecker
    {
      public:
        /**
         * @brief constructor for sensor class
         * @param nh [NodeHandlePtr const&] pointer to node's nodehandle
         * @param frameName [std::string const&] frame whose view is to be tracked
         */
        SpaceChecker(const NodeHandlePtr& nh, const std::string& frameName);

        /**
         * @override
         * @brief function that finds coverage, triggered when updating it
         * @param transform [tf::StampedTransform const&] tf that will be used
         * in coverage finding
         * @return void
         */
        virtual void findCoverage(const tf::StampedTransform& sensorTransform,
        const tf::StampedTransform& baseTransform);
        /**
         * @override
         * @brief publishes coverage map or patch
         * @return void
         */
        virtual void publishCoverage();

        /**
         * @brief Setter for static variable map2D_
         * @param map2D [boost::shared_ptr<nav_msgs::OccupancyGrid> const&] map
         * @return void
         */
        static void setMap2D(const boost::shared_ptr<nav_msgs::OccupancyGrid>& map2D)
        {
          map2D_ = map2D;
        }

        /**
         * @brief Deletes reference of variable 3DMap_
         * @return void
         */
        static void deleteMap()
        {
          map2D_.reset();
        }

        /**
         * @brief Setter for static variable OCCUPIED_CELL_THRES
         * @param occupiedCellThres [float] threshold
         * @return void
         */
        static void setOccupiedCellThres(float occupiedCellThres)
        {
          OCCUPIED_CELL_THRES = occupiedCellThres;
        }

      private:
        /**
         * @brief finds cell's space coverage as a percentage of the covered
         * space above it
         * @param cell [geometry_msgs::Point const&] cell in question
         * @param minHeight [double] minimun height of interest (base footprint)
         * @return float percentage of space covered by sensor.
         */
        float cellCoverage(const geometry_msgs::Point& cell, double minHeight);

        /**
         * @override
         * @brief Getter for sensor's parameters
         * @return void
         */
        virtual void getParameters();

      protected:
        //!< Global 2d map as it is sent by SLAM
        static boost::shared_ptr<nav_msgs::OccupancyGrid> map2D_;
        //!< Sensor's space coverage map
        nav_msgs::OccupancyGrid coveredSpace_;

        /*  Parameters  */
        //!< maximum height of interest
        double MAX_HEIGHT;
        static float OCCUPIED_CELL_THRES;

      private:
        friend class SpaceCheckerTest;
    };

}  // namespace pandora_sensor_coverage
}  // namespace pandora_data_fusion

#endif  // SENSOR_COVERAGE_SPACE_CHECKER_H

