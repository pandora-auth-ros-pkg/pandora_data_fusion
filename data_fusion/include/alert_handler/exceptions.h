// "Copyright [year] <Copyright Owner>"

#ifndef PANDORA_ROBOT_SRC_DATA_FUSION_INCLUDE_ALERT_HANDLER_EXCEPTIONS_H_
#define PANDORA_ROBOT_SRC_DATA_FUSION_INCLUDE_ALERT_HANDLER_EXCEPTIONS_H_

#include <stdexcept>
#include <string>

class AlertException : public std::runtime_error {
 public:
  explicit AlertException(const std::string errorDescription) :
     std::runtime_error(errorDescription) {}
};

#endif  // PANDORA_ROBOT_SRC_DATA_FUSION_INCLUDE_ALERT_HANDLER_EXCEPTIONS_H_
