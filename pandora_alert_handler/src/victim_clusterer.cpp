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
*   Christos Zalidis <zalidis@gmail.com>
*   Triantafyllos Afouras <afourast@gmail.com>
*********************************************************************/

#include "alert_handler/victim_clusterer.h"

VictimClusterer::VictimClusterer(float clusterRadius, float approachDist)
{  
  CLUSTER_RADIUS = clusterRadius;
  APPROACH_DIST = approachDist; 
}

/**
@details 
**/
VictimPtrVector VictimClusterer::createVictimList(
  const ObjectConstPtrVectorPtr& allObjects)
{  
  ObjectConstPtrVectorVector groupedObjects = groupObjects(allObjects);
    
  VictimPtrVector newVictimVector;

  for (int ii = 0; ii < groupedObjects.size(); ++ii)
  {
    VictimPtr newVictim(new Victim);

    newVictim->setObjects(groupedObjects[ii], APPROACH_DIST);
    
    newVictimVector.push_back(newVictim);
  }

  return newVictimVector;
}

/**
@details 
**/
ObjectConstPtrVectorVector 
  VictimClusterer::groupObjects(const ObjectConstPtrVectorPtr& allObjects)
{
  ObjectConstPtrVectorVector groupedObjects;

  for ( int objectIt = 0 ; objectIt < allObjects->size() ; ++objectIt )
  {
    ObjectConstPtr currentObj = allObjects->at(objectIt);

    bool isAdded = false;

    for (int ii = 0; ii < groupedObjects.size(); ++ii)
    {
      geometry_msgs::Point groupCenterPoint =
        findGroupCenterPoint(groupedObjects[ii]);

      double distance =
        Utils::distanceBetweenPoints2D(currentObj->
                                       getPose().position, groupCenterPoint);

      if (distance < CLUSTER_RADIUS)
      {
        groupedObjects[ii].push_back(currentObj);
        isAdded = true;
        break;
      }
    }

    if (!isAdded)
    {
      ObjectConstPtrVector newVect;
      newVect.push_back(currentObj);
      groupedObjects.push_back(newVect);
      isAdded = false;
    }
  }

  return groupedObjects;
}

/**
@details 
**/
geometry_msgs::Point VictimClusterer::findGroupCenterPoint(
  const ObjectConstPtrVector& objects)
{
  geometry_msgs::Point centerPoint;

  for (ObjectConstPtrVector::const_iterator it = objects.begin();
       it != objects.end(); ++it)
  {
    centerPoint.x += (*it)->getPose().position.x;
    centerPoint.y += (*it)->getPose().position.y;
  }

  centerPoint.x /= objects.size();
  centerPoint.y /= objects.size();

  return centerPoint;
}

/**
@details 
**/
void VictimClusterer::updateParams(float clusterRadius, float approachDist)
{
  CLUSTER_RADIUS = clusterRadius;
  APPROACH_DIST = approachDist;
}

