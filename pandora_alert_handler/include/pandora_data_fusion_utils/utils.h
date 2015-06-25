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
 *   Christos Zalidis <zalidis@gmail.com>
 *   Triantafyllos Afouras <afourast@gmail.com>
 *   Tsirigotis Christos <tsirif@gmail.com>
 *********************************************************************/

#ifndef PANDORA_DATA_FUSION_UTILS_UTILS_H
#define PANDORA_DATA_FUSION_UTILS_UTILS_H

#include <utility>
#include <cmath>
#include <boost/utility.hpp>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <tf/LinearMath/Vector3.h>

#include "pandora_data_fusion_utils/exceptions.h"
#include "pandora_data_fusion_utils/defines.h"

namespace pandora_data_fusion
{
namespace pandora_data_fusion_utils
{

  class Utils : private boost::noncopyable
  {
   public:
    static geometry_msgs::Point point2DAndHeight2Point3D(geometry_msgs::Point position, float height);
    static float distanceBetweenPoints2D(geometry_msgs::Point a, geometry_msgs::Point b);
    static float distanceBetweenPoints3D(geometry_msgs::Point a, geometry_msgs::Point b);
    static float distanceBetweenPoints(geometry_msgs::Point a, geometry_msgs::Point b, bool is3D);
    static bool arePointsInRange(geometry_msgs::Point pointA, geometry_msgs::Point pointB,
        bool is3D, float sensor_range);
    static bool isPoseInBox2D(const geometry_msgs::Pose& reference,
        double length, double width, const geometry_msgs::Pose& pose)
    static bool isOrientationClose(geometry_msgs::Quaternion orientA,
        geometry_msgs::Quaternion orientB,
        float diff_thres);
    static geometry_msgs::Quaternion calculateQuaternion(geometry_msgs::Point a,
        geometry_msgs::Point b);
    static geometry_msgs::Point vector3ToPoint(const tf::Vector3& vector);
    static float probabilityFromStdDev(float boundingRadius, float deviation);
    static float stdDevFromProbability(float boundingRadius, float probability);
  };

}  // namespace pandora_data_fusion_utils
}  // namespace pandora_data_fusion

#endif  // PANDORA_DATA_FUSION_UTILS_UTILS_H
