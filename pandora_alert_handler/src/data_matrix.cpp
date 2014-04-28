// "Copyright [year] <Copyright Owner>"

#include "alert_handler/data_matrix.h"

namespace pandora_data_fusion
{
  namespace pandora_alert_handler
  {

    DataMatrix::DataMatrix() {}

    PoseStamped DataMatrix::getPoseStamped() const
    {
      PoseStamped objPose = Object<DataMatrix>::getPoseStamped();
      objPose.header.frame_id = objPose.header.frame_id + "_" + content_;
      return objPose;
    }

    bool DataMatrix::isSameObject(const ObjectConstPtr& object) const
    {
      bool cond = Object<DataMatrix>::isSameObject(object) 
        && !content_.compare(
            boost::dynamic_pointer_cast<const DataMatrix>(object)->getContent());

      return cond;
    }

    void DataMatrix::getVisualization(visualization_msgs::
        MarkerArray* markers) const
    {
      visualization_msgs::Marker marker;

      marker.header.frame_id = getFrameId();
      marker.header.stamp = ros::Time::now();
      marker.ns = "DataMatrix";
      marker.id = id_;

      marker.pose = pose_;

      marker.type = visualization_msgs::Marker::SPHERE;

      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;

      marker.color.r = 0.2;
      marker.color.g = 0;
      marker.color.b = 1;
      marker.color.a = 0.7;

      markers->markers.push_back(marker);
    }

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

