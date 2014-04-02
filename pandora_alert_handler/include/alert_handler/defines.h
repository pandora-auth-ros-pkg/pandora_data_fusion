// "Copyright [year] <Copyright Owner>"

#ifndef PANDORA_ALERT_HANDLER_INCLUDE_ALERT_HANDLER_DEFINES_H_
#define PANDORA_ALERT_HANDLER_INCLUDE_ALERT_HANDLER_DEFINES_H_

#include <boost/math/constants/constants.hpp>

//!< Macro for pi.
#define PI boost::math::constants::pi<float>()

//!< Macro to convert to map coordinates from meters.
#define COORDS(X, Y, MAP) ceil((X - MAP->info.origin.position.x)\
    / MAP->info.resolution) + ceil((Y - MAP->info.origin.position.y)\
      / MAP->info.resolution) * MAP->info.width

#endif  // PANDORA_ALERT_HANDLER_INCLUDE_ALERT_HANDLER_DEFINES_H_
