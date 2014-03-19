#include "alert_handler/victim.h"
#include "gtest/gtest.h"


double pi= M_PI;

class VictimTest : public ::testing::Test {
 
  protected:

  VictimTest(){ }


  virtual void SetUp() {
    
    Victim1.reset( new Victim); 
    Victim2.reset( new Victim);
    Victim3.reset( new Victim);
    Victim4.reset( new Victim);
    Victim5.reset( new Victim);
    Victim6.reset( new Victim);
  } 
  
  virtual void TearDown()
  {
    
    clear();
  }
 //* helper functions 
 
// We create manually Tpa1(2,3,4) Tpa2(4,3,2) Hole1(1,2,0) (no rotation)
  void createVariousObjects1(ObjectConstPtrVector   &ObjConstPtrVectPtr1)
  {
    TpaPtr TpaPtr1(new Tpa);
    setPose(2,3,4,TpaPtr1);
    TpaPtr1->setType("tpa");
    TpaPtr TpaPtr2(new Tpa);
    setPose(4,3,2,TpaPtr2);
    TpaPtr2->setType("tpa");
    HolePtr HolePtr1(new Hole);
    setPose(1,2,0,HolePtr1);
    
    ObjConstPtrVectPtr1.push_back(TpaConstPtr(TpaPtr1));
    ObjConstPtrVectPtr1.push_back(TpaConstPtr(TpaPtr2));
    ObjConstPtrVectPtr1.push_back(HoleConstPtr(HolePtr1));
  } 
// We create manually Tpa1(2,3,4) Hole1(4,3,2) Hole2(0,4,2) Yaw=pi/4
  void createVariousObjects2(ObjectConstPtrVector   &ObjConstPtrVectPtr2)
  {
    TpaPtr TpaPtr1(new Tpa);
    setPose(2,3,4,TpaPtr1);
    TpaPtr1->setType("tpa");
    HolePtr HolePtr1(new Hole);
    setPose(4,3,2,HolePtr1,pi/4);
    HolePtr1->setType("hole");
    HolePtr HolePtr2(new Hole);
    setPose(0,4,2,HolePtr2);
    HolePtr2->setType("hole");
    ObjConstPtrVectPtr2.push_back(TpaConstPtr(TpaPtr1));
    ObjConstPtrVectPtr2.push_back(HoleConstPtr(HolePtr1));
    ObjConstPtrVectPtr2.push_back(HoleConstPtr(HolePtr2));
  } 
 

  
  /* accesing private functions*/
  
  void setPose ( int x,int y,int z,ObjectPtr Object,float yaw=0)
  {
    //z is set zero because this test is written for 2d
    geometry_msgs::Pose pose1;
    pose1.position.x=x;
    pose1.position.y=y;
    pose1.position.z=0;
    pose1.orientation=tf::createQuaternionMsgFromRollPitchYaw(0, 0, yaw);
    Object->setPose(pose1);
  }
  
   void updatePose(VictimPtr Victim,geometry_msgs::Pose poseVic,float dist)
  {
    Victim->updatePose(poseVic,dist);
  }
  
  
  int findRepresentativeObject(VictimPtr Victim)
  {
    return Victim->findRepresentativeObject();
  }
  
  void clear()
  {
    
    Victim::lastVictimId_=0;
  }
    
  /* Accesors to private variables*/
  
  
  int* getLastVictimId(VictimPtr Victim)
  {
    return &Victim->lastVictimId_;
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
  
  clear(); 
  Victim1.reset( new Victim); 
  EXPECT_FALSE(Victim1->getVisited());
  EXPECT_FALSE(Victim1->getValid());
  EXPECT_EQ(1,*getLastVictimId(Victim1));
  EXPECT_EQ(-1,Victim1->getSelectedObjectIndex());
 
    
  Victim2.reset( new Victim);
  EXPECT_FALSE(Victim1->getVisited());
  EXPECT_FALSE(Victim1->getValid());
  EXPECT_EQ(2,*getLastVictimId(Victim1));
  EXPECT_EQ(2,*getLastVictimId(Victim2));
  EXPECT_EQ(-1,Victim2->getSelectedObjectIndex());
  clear();
    
  }
    

TEST_F(VictimTest, isSameObject)
  {
  
  
  EXPECT_EQ(6,*getLastVictimId(Victim1));
  // MUST Be CHECKED ALSO FOR 3d WHEN SLAM BECAMES 3D
  setPose(5,1,0,Victim1);
  setPose(0,1,0,Victim2);
  EXPECT_TRUE(Victim1->isSameObject(VictimConstPtr(Victim2),  7));
  EXPECT_FALSE(Victim1->isSameObject(VictimConstPtr(Victim2),  -5));
  EXPECT_FALSE(Victim1->isSameObject(VictimConstPtr(Victim2),  1));
  clear();
  }

  
TEST_F(VictimTest, updatePose)
{
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




   
   
   TEST_F(VictimTest,setObjectsUfindRepresentativeObject)
  {
  
  ObjectConstPtrVector  ObjConstPtrVect1;
  // create the Objectvector and fill it with various objects 
  //Tpa1(2,3,4) Tpa2(4,3,2) Hole1(1,2,0) (yaw=0) ApproachDist=5
  createVariousObjects1((ObjConstPtrVect1));
  Victim1->setObjects(ObjConstPtrVect1,5);
  EXPECT_EQ(3,ObjConstPtrVect1.size());
  EXPECT_EQ(2,findRepresentativeObject(Victim1));
  EXPECT_EQ(1,Victim1->getPose().position.x);
  EXPECT_EQ(2,Victim1->getPose().position.y);
  EXPECT_EQ(0,Victim1->getPose().position.z);
  EXPECT_EQ(6,Victim1->getApproachPose().position.x);
  EXPECT_EQ(2,Victim1->getApproachPose().position.y);
  EXPECT_EQ(0,Victim1->getApproachPose().position.z);
  
  
  ObjectConstPtrVector  ObjConstPtrVect2;
  // create the Objectvector and fill it with various objects 
  //Tpa1(2,3,4) Hole1(4,3,2) Hole2(0,4,2) Yaw=pi/4 ApproachDist=6
  createVariousObjects2((ObjConstPtrVect2));
  Victim2->setObjects(ObjConstPtrVect2,6);
  EXPECT_EQ(3,ObjConstPtrVect2.size());
  EXPECT_EQ(1,findRepresentativeObject(Victim2));
  EXPECT_EQ(4,Victim2->getPose().position.x);
  EXPECT_EQ(3,Victim2->getPose().position.y);
  EXPECT_EQ(0,Victim2->getPose().position.z);
  EXPECT_FLOAT_EQ(8.24264,Victim2->getApproachPose().position.x);
  EXPECT_FLOAT_EQ(7.24264,Victim2->getApproachPose().position.y);
  EXPECT_EQ(0,Victim2->getApproachPose().position.z);
  
  }

  
  


//~ TEST_F(VictimTest,findRepresentativeObject)
   //~ {
     //~ 
     //~ 
     //~ 
    //~ }

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

    
 //~ 
  //~ void eraseObjectAt(int index, float approachDistance);
  //~ 
  //~ void sanityCheck(const ObjectConstPtrVectorPtr& allObjects,
            //~ float distThreshold, float approachDistance);
   //~ 
  //~ void addSensor(int sensorId);
         //~ 
//~ 
         //~ 
