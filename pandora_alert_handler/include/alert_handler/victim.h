// "Copyright [year] <Copyright Owner>"

#ifndef ALERT_HANDLER_VICTIM_H
#define ALERT_HANDLER_VICTIM_H

#include "alert_handler/objects.h"

namespace pandora_data_fusion
{
  namespace pandora_alert_handler
  {

    /**
     * @class Victim
     * @brief Class to describe a victim in the world
     * @details Inherits from Object
     */
    class Victim : public Object<Victim>
    {
      public:

        //!< Type definitions
        typedef boost::shared_ptr<Victim> Ptr;
        typedef boost::shared_ptr<Victim const> ConstPtr;

      public:

        /**
         * @brief Default constructor
         */
        Victim();

        /**
         * @brief Setter of the objects associated with the Victim. Selects and keeps
         * as many objects as the different types of objects comprising this victim.
         * @param objects [ObjectConstPtrVector const&] 
         * Group of objects considered a victim
         * @param approachDistance [float] 
         * Approach point's desired distance away from victim
         * @return void
         */
        void setObjects(const ObjectConstPtrVector& objects, 
            float approachDistance);

        /**
         * @brief Inspects this victim's objects in order to verify it. 
         * Sets its probability.
         * @return void
         */
        void inspect();

        /**
         * @brief Erases from victim's associated objects
         * @param index [int] Index in victim's vector of objects
         * @param approachDistance [float] 
         * Approach point's desired distance away from victim
         * @return void
         */
        void eraseObjectAt(int index, float approachDistance);

        /**
         * @override
         * @brief Getter for member pose_ (stamped)
         * @return PoseStamped pose_ (stamped)
         */
        virtual PoseStamped getPoseStamped() const;

        /**
         * @override
         * @brief Getter for geotiff information about the victim
         * @param res [data_fusion...::DatafusionGeotiffSrv::Response*]
         * @return void
         */
        virtual void fillGeotiff(data_fusion_communications::
            DatafusionGeotiffSrv::Response* res) const;

        /**
         * @override
         * @brief Getter for visualization markers of victim
         * @param markers [visualization_msgs::MarkerArray*] Pointer to marker array
         * @return void
         */
        virtual void getVisualization(visualization_msgs::MarkerArray* markers) const;

        /**
         * @brief Getter for member valid_
         * @return bool valid_
         */
        bool getValid() const
        {
          return valid_;
        }

        /**
         * @brief Getter for member visited_
         * @return bool visited_
         */
        bool getVisited() const
        {
          return visited_;
        }

        /**
         * @brief Getter for member selectedObjectIndex_
         * @return bool selectedObjectIndex_
         */
        int getSelectedObjectIndex() const
        {
          return selectedObjectIndex_;
        }

        /**
         * @brief Getter for member approachPose_
         * @return const Pose& approachPose_
         */
        const Pose& getApproachPose() const
        {
          return approachPose_;
        }

        /**
         * @brief Getter for member approachPose_ (stamped)
         * @return PoseStamped approachPose_ (stamped)
         */
        PoseStamped getApproachPoseStamped() const;

        /**
         * @brief Getter for member objects_
         * @return std::set<int>& objects_
         */
        const ObjectConstPtrVector& getObjects() const
        {
          return objects_;
        }

        /**
         * @brief Getter for victim's transform from /world (yaw-reversed)
         * @return tf::Transform
         */
        tf::Transform getRotatedTransform() const;

        /**
         * @brief Setter for member valid_
         * @param valid [bool] The new valid_ value
         * @return void
         */
        void setValid(bool valid)
        {
          valid_ = valid;
        }

        /**
         * @brief Setter for member visited_
         * @param visited [bool] The new visited_ value
         * @return void
         */
        void setVisited(bool visited)
        {
          visited_ = visited;
        }

      private:

        /**
         * @brief Updates position, orientation and approach pose
         * @param object [ObjectConstPtr const&] object containing new pose
         * @param approachDistance [float] distance of approch position from victim
         * @return void
         */
        void updatePose(const ObjectConstPtr& object, float approachDistance);

        /**
         * @brief Calculates the approach pose of the victim for it's current pose
         * @param objectType [std::string] representative object's type (hole or tpa)
         * @param approachDistance [float] The desired distance from the wall
         * @return Pose The approach pose
         */
        Pose calculateApproachPose(std::string objectType, 
            float approachDistance) const;

        /**
         * @brief Updates the representative object and consequently the pose 
         * @param approachDistance [float] The disired distance from the wall
         * @details Should be always called after any change on the objects_
         * @return void
         */
        void updateRepresentativeObject(float approachDistance);

        /**
         * @brief Selects the object to represent victim's pose out of victim's
         * grouped object
         * @param approachDistance [float] distance of approch position from victim
         * @return int index of selected object
         */
        template <class ObjectType>
          void findRepresentativeObject(const ObjectConstPtrVector& objects);

        /**
         * @brief Gets the transform from /world to the victim
         * @return tf::Transform
         */
        tf::Transform getTransform() const;

      protected:

        //!< The validity of the victim
        bool valid_;
        //!< True if the victim was visited false otherwise     
        bool visited_;
        //!< True if hole failed to pass validation and thus deleted
        bool holeDeleted_;
        //!< True if thermal failed to pass validation and thus deleted
        bool thermalDeleted_;

        //!< Index pointing to representative object
        int selectedObjectIndex_;

        //!< Pose with the approch point to the victim and its orientation
        geometry_msgs::Pose approachPose_;    

        //!< Victim's characteristic group of objects (Hole or Thermal)
        ObjectConstPtrVector objects_;

      private:

        static int lastVictimId_;  //!< The last in line victim ID

      private:

        friend class VictimTest;
        friend class VictimClustererTest;
    };

    typedef Victim::Ptr VictimPtr;
    typedef Victim::ConstPtr VictimConstPtr;
    typedef std::vector<VictimPtr> VictimPtrVector;

    template <class ObjectType>
      void Victim::findRepresentativeObject(const ObjectConstPtrVector& objects)
      {
        ObjectConstPtrVector::const_iterator objectIt = objects.end();
        float maxObjectProbability = 0;

        for(ObjectConstPtrVector::const_iterator it = objects.begin(); 
            it != objects.end(); it++)
        {
          if((*it)->getType() == ObjectType::getObjectType() && 
              (*it)->getProbability() > maxObjectProbability)
          {
            maxObjectProbability = (*it)->getProbability();
            objectIt = it;
          }
        }

        ObjectPtr representativeObject( new ObjectType );

        if(objectIt != objects.end())
          *representativeObject = *(*objectIt);

        for ( ObjectConstPtrVector::const_iterator it = objects.begin(); 
            it != objects.end(); it++)
        {
          if ((*it)->getType() == ObjectType::getObjectType() && it != objectIt)
          {
            representativeObject->update((*it));
          }
        }

        if (objectIt != objects.end())
          objects_.push_back(representativeObject);
      }

  }  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

#endif  // ALERT_HANDLER_VICTIM_H
