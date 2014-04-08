// "Copyright [year] <Copyright Owner>"

#ifndef ALERT_HANDLER_EXCEPTIONS_H
#define ALERT_HANDLER_EXCEPTIONS_H

#include <stdexcept>
#include <string>

class AlertException : public std::runtime_error
{
 public:

  explicit AlertException(const std::string errorDescription) :
     std::runtime_error(errorDescription) {}

};

#endif  // ALERT_HANDLER_EXCEPTIONS_H
