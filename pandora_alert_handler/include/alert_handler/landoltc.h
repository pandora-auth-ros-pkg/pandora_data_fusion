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
        typedef boost::shared_ptr< ObjectList<Landoltc> > ListPtr;
        typedef boost::shared_ptr< const ObjectList<Landoltc> > ListConstPtr;

      public:

        /**
         * @brief Constructor
         */
        Landoltc();

        virtual bool isSameObject(const ObjectConstPtr& object) const;

        virtual PoseStamped getPoseStamped() const;

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
    typedef Landoltc::ListPtr LandoltcListPtr;
    typedef Landoltc::ListConstPtr LandoltcListConstPtr;

  }  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

#endif  // ALERT_HANDLER_LANDOLTC_H
