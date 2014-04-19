// "Copyright [2014] <Chamzas Konstantinos>"
#include "alert_handler/victim_list.h"
#include "gtest/gtest.h"

namespace pandora_data_fusion
{
namespace pandora_alert_handler
{

class VictimListTest : public ::testing::Test {
 
 protected:

  VictimListTest() : victimList1(), victimList2(2, 1, 1.5), 
  victim1(new Victim), victim2( new Victim), victim3( new Victim), 
  victim4( new Victim), victim5( new Victim), victim6( new Victim) {}

  virtual void SetUp()
  {
     
     
    filterModelPtr.reset( new FilterModel );
    Victim::setHoleModel(filterModelPtr);
    Victim::setTpaModel(filterModelPtr);
    
    createVariousObjects1(&ObjConstPtrVect1);
    createVariousObjects2(&ObjConstPtrVect2);
    createVariousObjects3(&ObjConstPtrVect3);
    createVariousObjects4(&ObjConstPtrVect4);
     
    fillVictim(victim1, ObjConstPtrVect1);
    fillVictim(victim2, ObjConstPtrVect2);
    fillVictim(victim3, ObjConstPtrVect3);
    fillVictim(victim4, ObjConstPtrVect4);
     
    fillVictimList(&victimList1);
    
  } 
  
  virtual void TearDown()
  {
    victimList1.clear();
    
  }
  
  
// helper functions
void fillVictimList( VictimList* victimList1 )
  {
    victimList1->clear();
    victimList1->add(victim1);
    victimList1->add(victim2);
    victimList1->add(victim3);
  }
  
void fillVictim(VictimPtr Victim, ObjectConstPtrVector ObjConstPtrVect)
{
  Victim->setObjects(ObjConstPtrVect, 5);
}

// Tpa1(-1, 0, 0)  Tpa2(1, 0, 0) Hole1(0 , 1, 0)  
  void createVariousObjects1(ObjectConstPtrVector*   ObjConstPtrVect)
  {
    TpaPtr TpaPtr1(new Tpa);
    setPose(-1, 0, 0, TpaPtr1);
    TpaPtr TpaPtr2(new Tpa);
    setPose(1, 0, 0, TpaPtr2);
    HolePtr HolePtr1(new Hole);
    setPose(0, 1, 0, HolePtr1);
    ObjConstPtrVect->push_back(TpaConstPtr(TpaPtr1));
    ObjConstPtrVect->push_back(TpaConstPtr(TpaPtr2));
    ObjConstPtrVect->push_back(HoleConstPtr(HolePtr1));
  }
// Tpa1(2, 3, 0) Hole1(3, 3, 0) Hole2(2, 2.5, 0) 
  void createVariousObjects2(ObjectConstPtrVector*   ObjConstPtrVect)
  {
    TpaPtr TpaPtr1(new Tpa);
    setPose(2, 3, 0, TpaPtr1);
    HolePtr HolePtr1(new Hole);
    setPose(3, 3, 0, HolePtr1);
    HolePtr HolePtr2(new Hole);
    setPose(2, 2.5, 0, HolePtr2);
    ObjConstPtrVect->push_back(TpaConstPtr(TpaPtr1));
    ObjConstPtrVect->push_back(HoleConstPtr(HolePtr1));
    ObjConstPtrVect->push_back(HoleConstPtr(HolePtr2));
  }
// Tpa1(2.7, 3, 0) Tpa2(3, 3, 0) 
  void createVariousObjects3(ObjectConstPtrVector*   ObjConstPtrVect)
  {
    TpaPtr TpaPtr1(new Tpa);
    setPose(2.7, 3, 0, TpaPtr1);
    TpaPtr TpaPtr2(new Tpa);
    setPose(3, 3, 0, TpaPtr2);
    ObjConstPtrVect->push_back(TpaConstPtr(TpaPtr1));
    ObjConstPtrVect->push_back(TpaConstPtr(TpaPtr2));
  } 
  
// Tpa1(3, 3, 0) Hole1(10, 3, 0) 
  void createVariousObjects4(ObjectConstPtrVector*   ObjConstPtrVect)
  {
    TpaPtr TpaPtr1(new Tpa);
    setPose(3, 3, 0, TpaPtr1);
    HolePtr HolePtr1(new Hole);
    setPose(10, 3, 0, HolePtr1);
    ObjConstPtrVect->push_back(TpaConstPtr(TpaPtr1));
    ObjConstPtrVect->push_back(HoleConstPtr(HolePtr1));
  } 
  
  void setPose ( float x, float y, float z, ObjectPtr Object, float yaw =0)
  {
    // z is set zero because this test is written for 2d
    geometry_msgs::Pose pose1;
    pose1.position.x = x;
    pose1.position.y = y;
    pose1.position.z = 0;
    pose1.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, yaw);
    Object->setPose(pose1);
    Object->initializeObjectFilter(0.5, 0.5, 0.5); 
  }
// accesing private variables
  VictimList::iterator getCurrentVictimIt(VictimList victimList1)
  {
    return victimList1.currentVictimIt_;
  }
 //!< The pose of the currently tracked victim when tracking was last updated 
  geometry_msgs::Pose getCurrentApproachPose(VictimList victimList1)
  {
      return victimList1.currentApproachPose_;
  }
  //!< A map containing correspondence of victims returned indices with ids
  std::map<int, int> getMap(VictimList victimList1)
  {
    return victimList1.victimIndicesMap_;
  }
  //!< True if the victims were requested and given, false otherwise
  bool getVictimsRequestAndGiven(VictimList victimList1)
  {
     return victimList1.victimsRequestedAndGiven_;
  }
  //!< True if the victims were requested and given, false otherwise
  bool getCurrentVictimDied(VictimList victimList1)
  {
    return victimList1.currentVictimDied_;
  }
  
  //!< The distance of the approach pose from the wall
  float getApproachDist(VictimList victimList1)
  { 
     return victimList1.APPROACH_DIST;
  }
  //!< The approach pose distance threshold for informing fsm of change 
  float getVictimUpdate(VictimList victimList1)
  {
      return victimList1.VICTIM_UPDATE;
  }

// variables 
VictimList victimList1;
VictimList victimList2;
FilterModelPtr filterModelPtr;
VictimPtr victim1;
VictimPtr victim2;
VictimPtr victim3;
VictimPtr victim4;
VictimPtr victim5;
VictimPtr victim6;
ObjectConstPtrVector   ObjConstPtrVect1;
ObjectConstPtrVector   ObjConstPtrVect2;
ObjectConstPtrVector   ObjConstPtrVect3;
ObjectConstPtrVector   ObjConstPtrVect4;

};


TEST_F(VictimListTest, Constructor)
{
  EXPECT_FALSE(getVictimsRequestAndGiven(victimList1));
  EXPECT_FALSE(getCurrentVictimDied( victimList1));
  EXPECT_EQ(0.5, getApproachDist(victimList1));
  EXPECT_EQ(0.5, getVictimUpdate(victimList1));
  


  EXPECT_FALSE(getVictimsRequestAndGiven(victimList2));
  EXPECT_FALSE(getCurrentVictimDied( victimList2));
  EXPECT_EQ(1, getApproachDist(victimList2));
  EXPECT_EQ(1.5, getVictimUpdate(victimList2));

}


TEST_F(VictimListTest, contains)
{
  
  ASSERT_EQ(2, victimList1.size());
  EXPECT_TRUE(victimList1.contains(VictimConstPtr(victim1)));
  EXPECT_TRUE(victimList1.contains(VictimConstPtr(victim2)));
  EXPECT_TRUE(victimList1.contains(VictimConstPtr(victim3)));
  EXPECT_FALSE(victimList1.contains(VictimConstPtr(victim4)));
}

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

