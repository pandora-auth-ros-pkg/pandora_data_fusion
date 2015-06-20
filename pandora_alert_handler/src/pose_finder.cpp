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

#include <utility>
#include <vector>
#include <string>

#include "alert_handler/pose_finder.h"

namespace pandora_data_fusion
{
  namespace pandora_alert_handler
  {

    PoseFinder::PoseFinder(const MapPtr& map, const std::string& mapType)
      : map_(map)
    {
      listener_.reset(TfFinder::newTfListener(mapType));
    }

    void PoseFinder::updateParams(float occupiedCellThres,
        float heightHighThres, float heightLowThres,
        float orientationDist, float orientationCircle)
    {
      OCCUPIED_CELL_THRES = occupiedCellThres;
      HEIGHT_HIGH_THRES = heightHighThres;
      HEIGHT_LOW_THRES = heightLowThres;
      ORIENTATION_CIRCLE = orientationCircle;
      ORIENTATION_DIST = orientationDist;
    }

    Pose PoseFinder::findAlertPose(float alertYaw, float alertPitch,
        const tf::Transform& tfTransform)
    {
      Pose outPose;

      tf::Quaternion alertOrientation, sensorOrientation;
      tfTransform.getBasis().getRotation(sensorOrientation);
      tf::Vector3 origin = tfTransform.getOrigin();

      // Should be in compliance with how vision creates yaw and pitch
      // FAULTY CODE

      alertOrientation.setRPY(0, alertPitch, alertYaw);
      tf::Transform newTf(sensorOrientation * alertOrientation, origin);

      Point position = positionOnWall(newTf);

      Point framePosition = Utils::vector3ToPoint(origin);
      float distFromAlert = Utils::distanceBetweenPoints2D(
          position, framePosition);

      float height = calcHeight(newTf, distFromAlert);

      outPose.position = Utils::point2DAndHeight2Point3D(position, height);
      outPose.orientation = findAppropriateOrientation(framePosition, outPose.position);

      return outPose;
    }

    float PoseFinder::calcHeight(const tf::Transform& transform, float distFromAlert)
    {
      Point xDirection = Utils::vector3ToPoint(transform.getBasis().getColumn(0));
      float lengthInXYPlane = sqrt(xDirection.x * xDirection.x + xDirection.y * xDirection.y);
      float alertHeight = xDirection.z / lengthInXYPlane * distFromAlert;

      alertHeight += transform.getOrigin()[2];

      ROS_DEBUG_NAMED("pose_finder",
          "[ALERT_HANDLER]Height of alert = %f ", alertHeight);
      ROS_DEBUG_NAMED("pose_finder", "[ALERT_HANDLER]Distance from alert = %f ",
          distFromAlert);

      if (alertHeight > HEIGHT_HIGH_THRES || alertHeight < HEIGHT_LOW_THRES)
        throw AlertException("Alert either too low or two high");

      return alertHeight;
    }

    Point PoseFinder::positionOnWall(const tf::Transform& transform)
    {
      const float resolution = map_->info.resolution;
      float x = 0, y = 0, D = 5 * resolution;
      Point xDirection = Utils::vector3ToPoint(transform.getBasis().getColumn(0));
      // ROS_DEBUG("xDirection:\nx: %f\ny: %f\nz: %f\n", xDirection.x, xDirection.y, xDirection.z);

      float currX = transform.getOrigin()[0];
      float currY = transform.getOrigin()[1];
      // ROS_DEBUG("currX: %f, currY: %f", currX, currY);

      x = D * xDirection.x + currX;
      y = D * xDirection.y + currY;
      // ROS_DEBUG("x: %f, y: %f", x, y);

      // ROS_DEBUG("coords: %d", coords);
      while (CELL(x, y, map_) < OCCUPIED_CELL_THRES * 100)
      {
        x += resolution * xDirection.x;
        y += resolution * xDirection.y;
      }
      if (CELL(x, y, map_) > OCCUPIED_CELL_THRES * 100)
      {
        Point onWall;
        onWall.x = x;
        onWall.y = y;
        return onWall;
      }
      else
        throw AlertException("Can not find point on wall");
    }

    geometry_msgs::Quaternion PoseFinder::findAppropriateOrientation(
        const Point& framePoint, const Point& alertPoint)
    {
      std::vector<Point> points;
      float x = 0, y = 0;

      for (unsigned int i = 0; i < 360; i += 5)
      {
        x = alertPoint.x + ORIENTATION_CIRCLE * cos((i / 180.0) * PI);
        y = alertPoint.y + ORIENTATION_CIRCLE * sin((i / 180.0) * PI);

        if (CELL(x, y, map_)
            > OCCUPIED_CELL_THRES * 100)
        {
          Point temp;
          temp.x = x;
          temp.y = y;
          points.push_back(temp);
        }
      }

      std::pair<Point, Point> pointsOnWall = findDiameterEndPointsOnWall(points);

      float angle;

      //!< if points are too close, first point should be the
      //!< diametrically opposite of the second
      if (Utils::distanceBetweenPoints2D
          (pointsOnWall.first, pointsOnWall.second) < ORIENTATION_CIRCLE / 2)
      {
        angle = atan2((alertPoint.y - pointsOnWall.second.y),
            (alertPoint.x - pointsOnWall.second.x));

        Point onWall;
        onWall.x = alertPoint.x + ORIENTATION_CIRCLE * cos(angle);
        onWall.y = alertPoint.y + ORIENTATION_CIRCLE * sin(angle);
        pointsOnWall.first = onWall;
      }

      angle = atan2((pointsOnWall.second.y - pointsOnWall.first.y),
          (pointsOnWall.second.x - pointsOnWall.first.x));

      std::pair<Point, Point> approachPoints;

      Point first;
      first.x = alertPoint.x + ORIENTATION_DIST * cos((PI / 2) + angle);
      first.y = alertPoint.y + ORIENTATION_DIST * sin((PI / 2) + angle);
      approachPoints.first = first;

      Point second;
      second.x = alertPoint.x + ORIENTATION_DIST * cos((-PI / 2) + angle);
      second.y = alertPoint.y + ORIENTATION_DIST * sin((-PI / 2) + angle);
      approachPoints.second = second;

      if (Utils::distanceBetweenPoints2D(framePoint, approachPoints.first) <
          Utils::distanceBetweenPoints2D(framePoint, approachPoints.second))
      {
        return Utils::calculateQuaternion(alertPoint, approachPoints.first);
      }
      else
      {
        return Utils::calculateQuaternion(alertPoint, approachPoints.second);
      }
    }

    std::pair<Point, Point> PoseFinder::findDiameterEndPointsOnWall(
        std::vector<Point> points)
    {
      if (points.size() < 2)
      {
        throw AlertException("Can not calculate approach point");
      }

      float maxDist = 0, dist = 0;

      std::pair<Point, Point> pointsOnWall;

      for (unsigned int i = 0; i < points.size(); i++)
      {
        for (unsigned int j = i + 1; j < points.size(); j++)
        {
          dist = Utils::distanceBetweenPoints2D(points[i], points[j]);
          if (dist > maxDist)
          {
            maxDist = dist;
            pointsOnWall = std::make_pair(points[i], points[j]);
          }
        }
      }

      return pointsOnWall;
    }


    tf::Transform PoseFinder::lookupTransformFromWorld(const std_msgs::Header& header)
    {
      tf::StampedTransform tfTransform;

      listener_->waitForTransform(BaseObject::getGlobalFrame(), header.frame_id,
          header.stamp, ros::Duration(1));

      listener_->lookupTransform(BaseObject::getGlobalFrame(), header.frame_id,
          header.stamp, tfTransform);

      return tfTransform;
    }

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion
