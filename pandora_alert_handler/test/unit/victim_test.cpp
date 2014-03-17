#include "alert_handler/victim.h"
#include "gtest/gtest.h"


 

class VictimTest : public ::testing::Test {
 
  protected:

  VictimTest(){
    
                   }


  virtual void SetUp() {
    
  }
  
 
  
  /* helping functions*/
  
  //~ 
  void setPose ( int x,int y,int z,VictimPtr Victim)
  {
    geometry_msgs::Pose pose1;
    pose1.position.x=x;
    pose1.position.y=y;
    pose1.position.z=z;
    Victim->setPose(pose1);
  }
  
  void clear()
  {
    
    Victim::lastVictimId_=0;
  }
    
  /* Accesors*/
  
  
  int* getLastVictimId(VictimPtr Victim)
  {
    return &Victim->lastVictimId_;
  }
  
  void updatePose(VictimPtr Victim,geometry_msgs::Pose poseVic,float dist)
  {
    Victim->updatePose(poseVic,dist);
  }
  
 /* Variables */
 
  VictimPtr Victim1;
  VictimPtr Victim2;
  VictimPtr Victim3;
  VictimPtr Victim4;
  VictimPtr Victim5;
  VictimPtr Victim6;


};


//~ // Checks  if the Construstors behave Correctly 
TEST_F(VictimTest, Constructor)
  {
  
  Victim1.reset(new Victim);
  EXPECT_FALSE(Victim1->getVisited());
  EXPECT_FALSE(Victim1->getValid());
  EXPECT_EQ(1,*getLastVictimId(Victim1));
  EXPECT_EQ(-1,Victim1->getSelectedObjectIndex());
 
    
    
  Victim2.reset(new Victim);
  EXPECT_FALSE(Victim1->getVisited());
  EXPECT_FALSE(Victim1->getValid());
  EXPECT_EQ(2,*getLastVictimId(Victim1));
  EXPECT_EQ(2,*getLastVictimId(Victim2));
  EXPECT_EQ(-1,Victim2->getSelectedObjectIndex());
  clear();
    
  }
    

TEST_F(VictimTest, isSameObject)
  {
  
  Victim1.reset(new Victim);
  Victim2.reset(new Victim);
  EXPECT_EQ(2,*getLastVictimId(Victim1));
  // MUST Be CHECKED ALSO FOR 3d WHEN SLAM BECAMES 3D
  setPose(5,1,0,Victim1);
  setPose(0,1,0,Victim2);
  EXPECT_TRUE(Victim1->isSameObject(VictimConstPtr(Victim2),  7));
  EXPECT_FALSE(Victim1->isSameObject(VictimConstPtr(Victim2),  -5));
  EXPECT_FALSE(Victim1->isSameObject(VictimConstPtr(Victim2),  1));
  clear();
  }

  
TEST_F(VictimTest, updateObject)
{
   Victim1.reset(new Victim);
   geometry_msgs::Pose poseVic;
   poseVic.orientation=tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
   poseVic.position.x=1;
   poseVic.position.y=0;
   poseVic.position.z=0;
   updatePose(Victim1,poseVic,5);
   EXPECT_EQ(1,Victim1->getPose().position.x);
   EXPECT_EQ(0,Victim1->getPose().position.y);
   EXPECT_EQ(0,Victim1->getPose().position.z);
   EXPECT_EQ(6,Victim1->getApproachPose().position.x);
   EXPECT_EQ(0,Victim1->getApproachPose().position.y);
   EXPECT_EQ(0,Victim1->getApproachPose().position.z);
   clear();
   
   
   
}



//~ setObjects (const ObjectConstPtrVector &objects, float approachDistance)
//~ {
  //~ 
  //~ 
  //~ 
//~ }
//~ 
//~ TEST_F(VictimTest,findRepresentativeObject)
   //~ {
     //~ 
     //~ 
     //~ 
    //~ }
//~ 
//~ 
//~ TEST_F(VictimTest,eraseObjectAt)
  //~ {
    //~ 
    //~ 
    //~ 
    //~ 
    //~ 
  //~ 
  //~ }

    

         

         
