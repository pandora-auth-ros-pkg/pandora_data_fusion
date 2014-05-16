#ifndef ALERT_HANDLER_FACE_H
#define ALERT_HANDLER_FACE_H

#include "alert_handler/kalman_object.h"

namespace pandora_data_fusion
{
  namespace pandora_alert_handler
  {

    /**
     * @class Face
     * @brief Concrete class representing a Face Object. Inherits from Object
     */ 
    class Face : public KalmanObject<Face>
    {
      public:

        //!< Type Definitions
        typedef boost::shared_ptr<Face> Ptr;
        typedef boost::shared_ptr<Face const> ConstPtr;
        typedef std::vector<Ptr> PtrVector;
        typedef boost::shared_ptr<PtrVector> PtrVectorPtr;

      public:

        /**
         * @brief Constructor
         */
        Face();

        virtual void getVisualization(visualization_msgs::MarkerArray* markers) const;
    };

    typedef Face::Ptr FacePtr;
    typedef Face::ConstPtr FaceConstPtr;
    typedef std::vector< FacePtr > FacePtrVector;
    typedef boost::shared_ptr< FacePtrVector > FacePtrVectorPtr;

  }  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

#endif  // ALERT_HANDLER_FACE_H
