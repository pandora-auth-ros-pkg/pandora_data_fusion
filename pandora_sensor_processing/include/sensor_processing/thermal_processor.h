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
 *   Tsirigotis Christos <tsirif@gmail.com>
 *********************************************************************/

#ifndef SENSOR_PROCESSING_THERMAL_PROCESSOR_H
#define SENSOR_PROCESSING_THERMAL_PROCESSOR_H

#include <string>
#include <boost/utility.hpp>

#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>

#include "sensor_msgs/Image.h"
#include "pandora_common_msgs/GeneralAlertMsg.h"
#include "pandora_sensor_processing/ThermalProcessorConfig.h"

namespace pandora_sensor_processing
{

  class ThermalProcessor : private boost::noncopyable
  {
    public:

      /**
       * @brief Constructor
       * @param ns [std::string const&] Has the namespace of the node.
       */
      explicit ThermalProcessor(const std::string& ns);

    private:

      /**
       * @brief callback to thermalSensorSubscriber_
       * @param msg [pandora_arm_hardware_interface::ThermalMsg const&] contains
       * thermal density ppm
       * @return void
       */
      void thermalSensorCallback(
          const sensor_msgs::Image& msg); 

      /**
       * @brief Finalizes alert_ message and publishes alert with alertPublisher_
       * @return void
       */
      void publishAlert();

      void initRosInterfaces();

    protected:

      ros::NodeHandle nh_;

      ros::Subscriber thermalSensorSubscriber_;
      ros::Publisher alertPublisher_;

      pandora_common_msgs::GeneralAlertMsg alert;
      
      dynamic_reconfigure::Server< ThermalProcessorConfig >
        dynReconfServer_;
  };

}  // namespace pandora_sensor_processing

#endif  // SENSOR_PROCESSING_THERMAL_PROCESSOR_H
