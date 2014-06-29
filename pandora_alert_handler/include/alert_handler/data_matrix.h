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

#ifndef ALERT_HANDLER_DATA_MATRIX_H
#define ALERT_HANDLER_DATA_MATRIX_H

#include "alert_handler/kalman_object.h"

namespace pandora_data_fusion
{
  namespace pandora_alert_handler
  {

    /**
     * @class DataMatrix
     * @brief Concrete class representing a DataMatrix Object. Inherits from Object
     */ 
    class DataMatrix : public KalmanObject<DataMatrix>
    {
      public:

        //!< Type Definitions
        typedef boost::shared_ptr<DataMatrix> Ptr;
        typedef boost::shared_ptr<DataMatrix const> ConstPtr;
        typedef std::vector<Ptr> PtrVector;
        typedef boost::shared_ptr<PtrVector> PtrVectorPtr;
        typedef ObjectList<DataMatrix> List;
        typedef boost::shared_ptr<List> ListPtr;
        typedef boost::shared_ptr< const ObjectList<DataMatrix> > ListConstPtr;

      public:

        /**
         * @brief Constructor
         */
        DataMatrix();

        virtual bool isSameObject(const ObjectConstPtr& object) const;

        virtual void getVisualization(visualization_msgs::
            MarkerArray* markers) const;

        /**
         * @brief Getter for member content_
         * @return std::string The DATAMATRIX's content
         */
        std::string getContent() const
        {
          return content_;
        }

        /**
         * @brief Setter for member content_
         * @return void
         */
        void setContent(std::string content)
        {
          content_ = content;
        }

        /**
         * @brief Getter for member timeFound_
         * @return ros::Time The DATAMATRIX's timeFound
         */
        ros::Time getTimeFound() const
        {
          return timeFound_;
        }

        /**
         * @brief Setter for member timeFound_
         * @return void
         */
        void setTimeFound(ros::Time timeFound)
        {
          timeFound_ = timeFound;
        }

      private:

        //!< The dataMatrix's content
        std::string content_;
        //!< The time when this dataMatrix was first found
        ros::Time timeFound_;
    };

    typedef DataMatrix::Ptr DataMatrixPtr;
    typedef DataMatrix::ConstPtr DataMatrixConstPtr;
    typedef DataMatrix::PtrVector DataMatrixPtrVector;
    typedef DataMatrix::PtrVectorPtr DataMatrixPtrVectorPtr;
    typedef DataMatrix::List DataMatrixList;
    typedef DataMatrix::ListPtr DataMatrixListPtr;
    typedef DataMatrix::ListConstPtr DataMatrixListConstPtr;

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

#endif  // ALERT_HANDLER_DATA_MATRIX_H
