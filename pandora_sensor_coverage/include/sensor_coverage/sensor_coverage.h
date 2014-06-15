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

#ifndef SENSOR_COVERAGE_SENSOR_COVERAGE_H
#define SENSOR_COVERAGE_SENSOR_COVERAGE_H

#include <string>
#include <boost/utility.hpp>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include "state_manager/state_client.h"
#include "pandora_sensor_coverage/SensorCoverageConfig.h"

namespace pandora_data_fusion
{
  namespace pandora_sensor_coverage
  {

    //!< Type Definitions
    typedef boost::shared_ptr<ros::NodeHandle> NodeHandlePtr;

    class SensorCoverage 
      : public StateClient, private boost::noncopyable
    {
      public:
        /**
         * @brief Constructor
         * @param ns [std::string const&] Has the namespace of the node.
         */
        explicit SensorCoverage(const std::string& ns);


        /**
         * @override
         * @brief Callback for every state change that occurs in state server.
         * It is used to make changes according to the state we are entering.
         * @param newState [int] number that indicates the state
         * @return void
         */
        void startTransition(int newState) {}

        /**
         * @override
         * @brief Callback that activates when state transition has occured for
         * all state clients.
         * @return void
         */
        void completeTransition() {}

        /**
         * @brief Dynamic reconfiguration callback
         * @param config [...::SensorCoverageConfig const&] struct containing new params
         * @param level [uint32_t] level value
         * @return void
         */
        void dynamicReconfigCallback(
            const ::pandora_sensor_coverage::SensorCoverageConfig& config,
            uint32_t level);

      private:
        /**
         * @brief Method for initializing ros interfaces: subscribers, publishers,
         * action clients, dynamic reconfiguration servers.
         * @return void
         */
        void initRosInterfaces();

      private:
        NodeHandlePtr nh_;

        dynamic_reconfigure::Server< ::pandora_sensor_coverage::SensorCoverageConfig >
          dynReconfServer_;
    };

}  // namespace pandora_sensor_coverage
}  // namespace pandora_data_fusion

#endif  // SENSOR_COVERAGE_SENSOR_COVERAGE_H
