#ifndef ALERT_HANDLER_HOLE_H
#define ALERT_HANDLER_HOLE_H

#include "alert_handler/kalman_object.h"

namespace pandora_data_fusion
{
  namespace pandora_alert_handler
  {

    /**
     * @class Hole
     * @brief Concrete class representing a Hole Object. Inherits from Object
     */ 
    class Hole : public KalmanObject<Hole>
    {
      public:

        //!< Type Definitions
        typedef boost::shared_ptr<Hole> Ptr;
        typedef boost::shared_ptr<Hole const> ConstPtr;

      public:

        /**
         * @brief Constructor
         */
        Hole();

        virtual void getVisualization(visualization_msgs::
            MarkerArray* markers) const;

        /**
         * @brief Getter for member holeId_
         * @return int The holeId 
         */
        unsigned int getHoleId() const 
        {
          return holeId_;
        }

        /**
         * @brief Setter for member holeId_
         * @return void
         */
        void setHoleId(int holeId) 
        {
          holeId_ = holeId;
        }

      private:

        //!< The hole's holeId. Caution: Not to be confused with the Object id!
        unsigned int holeId_;
    };

    typedef Hole::Ptr HolePtr;
    typedef Hole::ConstPtr HoleConstPtr;
    typedef std::vector< HolePtr > HolePtrVector;
    typedef boost::shared_ptr< HolePtrVector > HolePtrVectorPtr;

  }  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

#endif  // ALERT_HANDLER_HOLE_H
