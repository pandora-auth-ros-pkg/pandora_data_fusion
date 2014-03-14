#include "alert_handler/objects.h"
#include "gtest/gtest.h"



namespace {
 

class ObjectsTest : public ::testing::Test {
 
  protected:

  ObjectsTest(){
    
    Qr2Ptr.reset(new Qr);
    Hazmat2Ptr.reset(new Hazmat);
    Hazmat3Ptr.reset(new Hazmat);
  }


  virtual void SetUp() {
//The distance Between the points is 5 

    pose1.position.x=0;
    pose1.position.y=0;
    pose1.position.z=0;
    pose2.position.x=4;
    pose2.position.y=3;
    pose2.position.z=0;
    Qr1.setPose(pose1);
    Qr2Ptr->setPose(pose2);
    Hazmat1.setPose(pose1);
    Hazmat1.setPattern(1);
    Hazmat2Ptr->setPose(pose2);
    Hazmat2Ptr->setPattern(1);
    Hazmat3Ptr->setPose(pose2);
    Hazmat3Ptr->setPattern(0);
  }
 /* Variables */
  geometry_msgs::Pose pose1;
  geometry_msgs::Pose pose2;
  Qr Qr1;
  QrPtr Qr2Ptr;
  Hazmat Hazmat1;
  HazmatPtr Hazmat2Ptr;
  HazmatPtr Hazmat3Ptr;
};


// Checks  if isSameObject() behaves correctly for all possible inputs(Qr) 
TEST_F(ObjectsTest, IsSameQr) {
  
  
  EXPECT_FALSE(Qr1.isSameObject( ObjectConstPtr(Qr2Ptr) , 4));
  EXPECT_FALSE(Qr1.isSameObject( ObjectConstPtr(Qr2Ptr) , -4));
  EXPECT_TRUE(Qr1.isSameObject( ObjectConstPtr(Qr2Ptr) , 6));
    }
         
// Checks  if isSameObject() behaves correctly for all possible inputs(Hazmat)     
TEST_F(ObjectsTest, IsSameHazmat) {
  EXPECT_FALSE(Hazmat1.isSameObject( ObjectConstPtr(Hazmat2Ptr) , 4));
  EXPECT_FALSE(Hazmat1.isSameObject( ObjectConstPtr(Hazmat2Ptr) , -4));
  EXPECT_FALSE(Hazmat1.isSameObject( ObjectConstPtr(Hazmat3Ptr) , 6));  
  EXPECT_TRUE(Hazmat1.isSameObject( ObjectConstPtr(Hazmat2Ptr) ,6));
    }
}  // namespace


int main(int argc, char **argv) {
  
  ros::Time::init();
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
