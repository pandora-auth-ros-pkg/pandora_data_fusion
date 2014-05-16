#ifndef ALERT_HANDLER_QR_H
#define ALERT_HANDLER_QR_H

#include "alert_handler/kalman_object.h"

namespace pandora_data_fusion
{
  namespace pandora_alert_handler
  {

    /**
     * @class Qr
     * @brief Concrete class representing a Qr Object. Inherits from Object
     */ 
    class Qr : public KalmanObject<Qr>
    {
      public:

        //!< Type Definitions
        typedef boost::shared_ptr<Qr> Ptr;
        typedef boost::shared_ptr<Qr const> ConstPtr;
        typedef std::vector<Ptr> PtrVector;
        typedef boost::shared_ptr<PtrVector> PtrVectorPtr;

      public:

        /**
         * @brief Constructor
         */
        Qr();

        virtual bool isSameObject(const ObjectConstPtr& object) const;

        virtual PoseStamped getPoseStamped() const;

        virtual void fillGeotiff(data_fusion_communications::
            DatafusionGeotiffSrv::Response* res) const;

        virtual void getVisualization(visualization_msgs::
            MarkerArray* markers) const;

        /**
         * @brief Getter for member content_
         * @return std::string The QR's content
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

      private:

        //!< The qr's content
        std::string content_;
        //!< The time when this qr was first found
        ros::Time timeFound_;
    };

    typedef Qr::Ptr QrPtr;
    typedef Qr::ConstPtr QrConstPtr;
    typedef std::vector< QrPtr > QrPtrVector;
    typedef boost::shared_ptr< QrPtrVector > QrPtrVectorPtr;

  }  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

#endif  // ALERT_HANDLER_QR_H
