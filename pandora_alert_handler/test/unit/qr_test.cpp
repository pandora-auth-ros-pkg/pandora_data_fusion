#include "alert_handler/objects.h"
#include "gtest/gtest.h"



namespace {
 

class QrTest : public ::testing::Test {
 
  protected:

  QrTest(){
    
    Qr2Ptr.reset(new Qr);
     
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
   
  }
 /* Variables */
       geometry_msgs::Pose pose1;
      geometry_msgs::Pose pose2;
      Qr Qr1;
      QrPtr Qr2Ptr;
};


// Checks  if isSameObject() behaves correctly for all possible inputs(Qr) 
TEST_F(QrTest, IsSameQr) {
  
  
  EXPECT_FALSE(Qr1.isSameObject( ObjectConstPtr(Qr2Ptr) , 4));
  EXPECT_FALSE(Qr1.isSameObject( ObjectConstPtr(Qr2Ptr) , -4));
  EXPECT_TRUE(Qr1.isSameObject( ObjectConstPtr(Qr2Ptr) , 6));
    }
}  // namespace


int main(int argc, char **argv) {
  
  ros::Time::init();
  ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
