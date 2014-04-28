// "Copyright [year] <Copyright Owner>"

#ifndef ALERT_HANDLER_DATAMATRIX_H
#define ALERT_HANDLER_DATAMATRIX_H

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
        typedef boost::shared_ptr< ObjectList<DataMatrix> > ListPtr;
        typedef boost::shared_ptr< const ObjectList<DataMatrix> > ListConstPtr;

      public:

        /**
         * @brief Constructor
         */
        DataMatrix();

        virtual bool isSameObject(const ObjectConstPtr& object) const;

        virtual PoseStamped getPoseStamped() const;

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
    typedef DataMatrix::ListPtr DataMatrixListPtr;
    typedef DataMatrix::ListConstPtr DataMatrixListConstPtr;

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

#endif  // ALERT_HANDLER_DATAMATRIX_H
