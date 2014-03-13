// "Copyright [year] <Copyright Owner>"

#ifndef PANDORA_ALERT_HANDLER_INCLUDE_ALERT_HANDLER_POSE_FINDER_H_
#define PANDORA_ALERT_HANDLER_INCLUDE_ALERT_HANDLER_POSE_FINDER_H_

#include <utility> 
#include <vector> 

#include <nav_msgs/OccupancyGrid.h>

#include <tf/transform_broadcaster.h>

#include "alert_handler/defines.h"
#include "alert_handler/tf_finder.h"
#include "alert_handler/tf_listener.h"
#include "alert_handler/objects.h"
#include "alert_handler/utils.h"

typedef nav_msgs::OccupancyGrid Map;
typedef nav_msgs::OccupancyGridPtr MapPtr;
typedef nav_msgs::OccupancyGridConstPtr MapConstPtr;

class PoseFinder {

 public:

  PoseFinder(const MapConstPtr& map, const std::string& mapType,
    float occupiedCellThres = 0.5,
    float heightHighThres = 1.2, float heightLowThres = 0,
    float approachDist = 0.5, int orientationDist = 20,
    int orientationCircle = 10);
  Pose findAlertPose(float alertYaw, float alertPitch,
    tf::Transform tfTransform);
  tf::Transform lookupTransformFromWorld(std_msgs::Header header);

  void updateParams(float occupiedCellThres,
    float heightHighThres, float heightLowThres,
    float approachDist,
    int orientationDist, int orientationCircle);

 private:

  Point positionOnWall(Point startPoint, float angle);
  float calcHeight(float alertPitch, float height, float distFromAlert);
  geometry_msgs::Quaternion findNormalVectorOnWall(Point framePoint,
      Point alertPoint);
  std::pair<Point, Point> findDiameterEndPointsOnWall(
      std::vector<Point> points);

  void publishVisionTransform(float alertYaw, float alertPitch,
      tf::Transform worldHeadCameraTransform);
      
 private:

  const MapConstPtr& map_;

  TfListenerPtr listener_;
  tf::TransformBroadcaster victimFrameBroadcaster;

  //params
  int ORIENTATION_CIRCLE;
  int ORIENTATION_DIST;
  float APPROACH_DIST;
  float HEIGHT_HIGH_THRES;
  float HEIGHT_LOW_THRES;
  float OCCUPIED_CELL_THRES;
};

typedef boost::scoped_ptr< PoseFinder > PoseFinderPtr;

#endif  // PANDORA_ALERT_HANDLER_INCLUDE_ALERT_HANDLER_POSE_FINDER_H_
