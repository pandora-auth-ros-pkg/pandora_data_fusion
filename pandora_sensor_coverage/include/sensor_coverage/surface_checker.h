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

#ifndef SENSOR_COVERAGE_SURFACE_CHECKER_H
#define SENSOR_COVERAGE_SURFACE_CHECKER_H

#include <string>
#include <boost/shared_ptr.hpp>

#include "octomap/ColorOcTree.h"

#include "sensor_coverage/coverage_checker.h"

namespace pandora_data_fusion
{
  namespace pandora_sensor_coverage
  {

    /**
     * @brief class that contains methods to find surface coverage.
     * Keeps surface coverage patch.
     */
    class SurfaceChecker : public CoverageChecker
    {
      public:
        /**
         * @brief Constructor for surface checker class
         * @param nh [NodeHandlePtr const&] pointer to node's nodehandle
         * @param frameName [std::string const&] frame whose view is to be tracked
         */
        SurfaceChecker(const NodeHandlePtr& nh, const std::string& frameName);

        /**
         * @override
         * @brief function that finds coverage, triggered when updating it
         * @param transform [tf::StampedTransform const&] tf that will be used
         * in coverage finding
         * @return void
         */
        virtual void findCoverage(const tf::StampedTransform& transform);

        /**
         * @override
         * @brief publishes coverage map or patch
         * @return void
         */
        virtual void publishCoverage();

      private:
        /**
         * @brief finds at map3D_ the coverage of a point on a surface 
         * as the product of direction with surface's normal vector.
         * @param pointOnWall [octomap::point3d const&] point in search
         * @param direction [octomap::point3d const&] direction of tracing ray
         * @return unsigned char estimation of point's coverage.
         */
        unsigned char findPointCoverage(const octomap::point3d& pointOnWall,
            const octomap::point3d& direction);

      protected:
        //!< Sensor's surface coverage patch
        boost::shared_ptr<octomap::ColorOcTree> coveredSurface_;

      private:
        friend class SurfaceCheckerTest;
    };

}  // namespace pandora_sensor_coverage
}  // namespace pandora_data_fusion

#endif  // SENSOR_COVERAGE_SURFACE_CHECKER_H

