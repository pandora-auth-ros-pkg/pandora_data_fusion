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

#ifndef ALERT_HANDLER_LANDOLTC_H
#define ALERT_HANDLER_LANDOLTC_H

#include "alert_handler/kalman_object.h"

namespace pandora_data_fusion
{
  namespace pandora_alert_handler
  {

    /**
     * @class Landoltc
     * @brief Concrete class representing a Landoltc Object. Inherits from Object
     */ 
    class Landoltc : public KalmanObject<Landoltc>
    {
      public:

        //!< Type Definitions
        typedef boost::shared_ptr<Landoltc> Ptr;
        typedef boost::shared_ptr<Landoltc const> ConstPtr;
        typedef std::vector<Ptr> PtrVector;
        typedef boost::shared_ptr<PtrVector> PtrVectorPtr;
        typedef ObjectList<Landoltc> List;
        typedef boost::shared_ptr<List> ListPtr;
        typedef boost::shared_ptr< const ObjectList<Landoltc> > ListConstPtr;

      public:

        /**
         * @brief Constructor
         */
        Landoltc();

        virtual void getVisualization(visualization_msgs::MarkerArray* markers) const;

        /**
         * @brief Getter for member pattern_
         * @return int The Landoltc's pattern
         */
        std::vector<float> getAngles() const
        {
          return angles_;
        }

        /**
         * @brief Setter for member angles
         * @return void
         */
        void setAngles(const std::vector<float>& angles)
        {
          angles_ = angles;
        }

      private:

        //!< The hazmat's pattern
        std::vector<float> angles_;
    };

    typedef Landoltc::Ptr LandoltcPtr;
    typedef Landoltc::ConstPtr LandoltcConstPtr;
    typedef Landoltc::PtrVector LandoltcPtrVector;
    typedef Landoltc::PtrVectorPtr LandoltcPtrVectorPtr;
    typedef Landoltc::List LandoltcList;
    typedef Landoltc::ListPtr LandoltcListPtr;
    typedef Landoltc::ListConstPtr LandoltcListConstPtr;

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

#endif  // ALERT_HANDLER_LANDOLTC_H
