// "Copyright [year] <Copyright Owner>"

#include "alert_handler/landoltc.h"

namespace pandora_data_fusion
{
  namespace pandora_alert_handler
  {

    Landoltc::Landoltc() {}

    PoseStamped Landoltc::getPoseStamped() const
    {
      PoseStamped objPose = Object::getPoseStamped();
      objPose.header.frame_id = objPose.header.frame_id + 
        "_" + boost::to_string(angles_.size());
      return objPose;
    }

    bool Landoltc::isSameObject(const ObjectConstPtr& object) const
    {
      bool cond = false;
      float epsilon = 0.2;

      if(!Object<Landoltc>::isSameObject(object))
        return false;
      std::vector<float> angles = boost::dynamic_pointer_cast<
        const Landoltc>(object)->getAngles();
      if(angles_.size() == angles.size())
      {
        for(int ii = 0; ii < angles.size(); ++ii)
        {
          cond = (angles[ii] - angles_[ii] > -epsilon) &&
            (epsilon > angles[ii] - angles_[ii]);
          if(!cond)
            return false;
        }
      }
      return true;
    } 

    void Landoltc::getVisualization(visualization_msgs::MarkerArray* markers) const
    {
      visualization_msgs::Marker marker;

      marker.header.frame_id = "/world";
      marker.header.stamp = ros::Time::now();
      marker.ns = "Landoltc";
      marker.id = id_;

      marker.pose = pose_;

      marker.type = visualization_msgs::Marker::CUBE;

      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;

      marker.color.r = 0.80;
      marker.color.g = 0;
      marker.color.b = 0.4;
      marker.color.a = 0.7;

      markers->markers.push_back(marker);
    }

  }  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

