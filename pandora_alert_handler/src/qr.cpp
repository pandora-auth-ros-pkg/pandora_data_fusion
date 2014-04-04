// "Copyright [year] <Copyright Owner>"

#include "alert_handler/objects.h"

Qr::Qr() {
  type_ = "qr";
  timeFound_ = ros::Time::now();
}

geometry_msgs::PoseStamped Qr::getPoseStamped() const {

  geometry_msgs::PoseStamped objPose = Object::getPoseStamped();
  objPose.header.frame_id = "qr_" + boost::to_string(id_) + "_" + content_;
  return objPose;
}

bool Qr::isSameObject(const ObjectConstPtr& object, float distance) const {

  bool cond = false;

  if(object->getType().compare(std::string("tpa"))) {
    cond = Object::isSameObject(object, distance);
    if (!object->getType().compare(type_)) {
      cond = cond &&
        !content_.compare(
          boost::dynamic_pointer_cast<const Qr>(object)->getContent());
    }
  }

  return cond;

}

void Qr::fillGeotiff(
  data_fusion_communications::DatafusionGeotiffSrv::Response* res) const {

  res->qrx.push_back( pose_.position.x );
  res->qry.push_back( pose_.position.y );
  res->qrworldx.push_back( pose_.position.x );
  res->qrworldy.push_back( pose_.position.y );
  res->qrcontent.push_back(content_);
  res->qrtimestamp.push_back(timeFound_);
}

void Qr::getVisualization(visualization_msgs::MarkerArray* markers) const {

  visualization_msgs::Marker marker;

  marker.header.frame_id = "/world";
  marker.header.stamp = ros::Time::now();
  marker.ns = "Qr";
  marker.id = id_;

  marker.pose = pose_;

  marker.type = visualization_msgs::Marker::SPHERE;

  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;

  marker.color.r = 0;
  marker.color.g = 0;
  marker.color.b = 1;
  marker.color.a = 0.7;

  markers->markers.push_back(marker);
}

