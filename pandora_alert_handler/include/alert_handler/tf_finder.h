// "Copyright [year] <Copyright Owner>"

#ifndef PANDORA_ALERT_HANDLER_INCLUDE_ALERT_HANDLER_TF_FINDER_H_
#define PANDORA_ALERT_HANDLER_INCLUDE_ALERT_HANDLER_TF_FINDER_H_

#include <string>

#include "alert_handler/exceptions.h"
#include "alert_handler/utils.h"
#include "alert_handler/tf_listener.h"


class TfFinder {

 public:
 
  static TfListener* newTfListener(const std::string& type) {

    if( type == "SLAM" ) {
      return new RosTfListener;
    }
    if( type == "TEST" ) {
      return new TfListener;
    }
    else {
      ROS_ERROR("[ALERT_HANDLER %d]Runtime error: map_type not found.", __LINE__);
      return NULL;
    }
  }

 private:

  TfFinder() {}
 
};

#endif  // PANDORA_ALERT_HANDLER_INCLUDE_ALERT_HANDLER_TF_FINDER_H_
