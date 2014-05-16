#ifndef ALERT_HANDLER_HAZMAT_H
#define ALERT_HANDLER_HAZMAT_H

#include "alert_handler/kalman_object.h"

namespace pandora_data_fusion
{
  namespace pandora_alert_handler
  {

    /**
     * @class Hazmat
     * @brief Concrete class representing a Hazmat Object. Inherits from Object
     */ 
    class Hazmat : public KalmanObject<Hazmat>
    {
      public:

        //!< Type Definitions
        typedef boost::shared_ptr<Hazmat> Ptr;
        typedef boost::shared_ptr<Hazmat const> ConstPtr;
        typedef std::vector<Ptr> PtrVector;
        typedef boost::shared_ptr<PtrVector> PtrVectorPtr;

      public:

        /**
         * @brief Constructor
         */
        Hazmat();

        virtual bool isSameObject(const ObjectConstPtr& object) const;

        virtual PoseStamped getPoseStamped() const;

        virtual void fillGeotiff(data_fusion_communications::
            DatafusionGeotiffSrv::Response* res) const;

        virtual void getVisualization(visualization_msgs::MarkerArray* markers) const;

        /**
         * @brief Getter for member pattern_
         * @return int The Hazmat's pattern
         */
        int getPattern() const
        {
          return pattern_;
        }

        /**
         * @brief Setter for member pattern_
         * @return void
         */
        void setPattern(int pattern)
        {
          pattern_ = pattern;
        }

      private:

        //!< The hazmat's pattern
        int pattern_;
    };

    typedef Hazmat::Ptr HazmatPtr;
    typedef Hazmat::ConstPtr HazmatConstPtr;
    typedef std::vector< HazmatPtr > HazmatPtrVector;
    typedef boost::shared_ptr< HazmatPtrVector > HazmatPtrVectorPtr;

  }  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

#endif  // ALERT_HANDLER_HAZMAT_H
