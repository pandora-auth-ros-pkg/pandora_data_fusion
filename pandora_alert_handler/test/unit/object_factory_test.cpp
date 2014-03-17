#include "alert_handler/object_factory.h"
#include "gtest/gtest.h"
#include <ctime> 
#include <math.h>
 

double pi= M_PI;
class ObjectFactoryTest : public ::testing::Test
{ 
  protected: 
  
  ObjectFactoryTest(): map_type1("TEST"),MapPtr1(new Map) 
  { 
  
    createSimpleMap(MapPtr1);
    ObjectFactoryPtr1.reset(new ObjectFactory(MapPtr1,map_type1,0.5,1.2,0,0.5,20,0)); 
    
  }
    virtual void SetUp() 
  { 
    createHoleVector();
    createHazmatVector();
    createQrVector();
    
  }



 

  void createHoleVector()
  {
    HoleDirVect1.header.seq= 1; 
    HoleDirVect1.header.frame_id= "Maria";
    HoleDir1.yaw = -pi/6;
    HoleDir1.pitch =0; 
    HoleDir1.probability = 4;
    HoleDir1.holeId = 1; 
    HoleDirVect1.holesDirections.push_back(HoleDir1); 
    HoleDir2.yaw =-pi/3;
    HoleDir2.pitch = 0; 
    HoleDir2.probability = 0; 
    HoleDir2.holeId = 2;
    HoleDirVect1.holesDirections.push_back(HoleDir2);
    HoleDir3.yaw = pi/4; 
    HoleDir3.pitch = pi/6*(0.1); 
    HoleDir3.probability = 0; 
    HoleDir3.holeId = 3;
    HoleDirVect1.holesDirections.push_back(HoleDir3);
  }
 
 void createQrVector()
  {
   
    Qr1.yaw = -pi/6;
    Qr1.pitch =0;
    Qr1.QRcontent = "Danger"; 
    QrVect1.qrAlerts.push_back(Qr1); 
    Qr2.yaw =-pi/3;
    Qr2.pitch = 0; 
    Qr2.QRcontent ="Ultra Danger"; 
    QrVect1.qrAlerts.push_back(Qr2);
    Qr3.QRcontent = pi/3; 
    Qr3.pitch = pi/6*(0.1);
    Qr3.QRcontent ="The victim is here";
    QrVect1.qrAlerts.push_back(Qr3);
  }
 
 void createHazmatVector()
  {

    Hazmat1.yaw = -pi/3;
    Hazmat1.pitch = pi/60; 
    Hazmat1.patternType = 1; 
    HazmatVect1.hazmatAlerts.push_back(Hazmat1); 
    Hazmat2.yaw = pi/4;
    Hazmat2.pitch = pi/30; 
    Hazmat2.patternType = 2;
    HazmatVect1.hazmatAlerts.push_back(Hazmat2);
    Hazmat3.yaw = pi/6;  
    Hazmat3.pitch = pi/40; 
    Hazmat3.patternType = 3;
    HazmatVect1.hazmatAlerts.push_back(Hazmat3);
  }
 
 
  void  createSimpleMap(MapPtr MapPtr1)
  {
    MapPtr1->info.width=1;
    MapPtr1->info.height=1;
    MapPtr1->info.origin.position.x = 0;
    MapPtr1->info.origin.position.y = 0;
    MapPtr1->info.origin.position.z = 0;
    MapPtr1->info.origin.orientation.x = 0;
    MapPtr1->info.origin.orientation.y = 0;
    MapPtr1->info.origin.orientation.z = 0;
    MapPtr1->info.origin.orientation.w = 0;
    for ( int i=0 ;i<100; i++)
    {
      MapPtr1->data.push_back(50);
           
    }
  }
    
    
    /* variables */

  MapPtr MapPtr1;
  const std::string map_type1;
  ObjectFactoryPtr ObjectFactoryPtr1;
  vision_communications::HolesDirectionsVectorMsg HoleDirVect1;
  vision_communications::HoleDirectionMsg HoleDir1; 
  vision_communications::HoleDirectionMsg HoleDir2; 
  vision_communications::HoleDirectionMsg HoleDir3;
  vision_communications::QRAlertsVectorMsg QrVect1; 
  vision_communications::QRAlertMsg Qr1;
  vision_communications::QRAlertMsg Qr2;
  vision_communications::QRAlertMsg Qr3;
  vision_communications::HazmatAlertsVectorMsg HazmatVect1;
  vision_communications::HazmatAlertMsg Hazmat1;
  vision_communications::HazmatAlertMsg Hazmat2;
  vision_communications::HazmatAlertMsg Hazmat3;
  data_fusion_communications::ThermalDirectionAlertMsg  TpaDir1;
    
};
    
    


TEST_F(ObjectFactoryTest,makeHoles)
  { 
    HolePtrVectorPtr holeVec1(new HolePtrVector);
    holeVec1=ObjectFactoryPtr1-> makeHoles(HoleDirVect1);
    EXPECT_EQ( 3,holeVec1->size());
    EXPECT_TRUE(true); 
  } 
  
  

TEST_F(ObjectFactoryTest,makeQrs)
  { 
    QrPtrVectorPtr qrsVectorPtr( new QrPtrVector);
    qrsVectorPtr=ObjectFactoryPtr1-> makeQrs(QrVect1);
    EXPECT_EQ( 3,qrsVectorPtr->size());
    EXPECT_TRUE(true); 
  } 
  
  
TEST_F(ObjectFactoryTest,makeHazmats)
  { 
    HazmatPtrVectorPtr hazmatsVectorPtr( new HazmatPtrVector );
    hazmatsVectorPtr=ObjectFactoryPtr1-> makeHazmats(HazmatVect1);
    EXPECT_EQ( 3,hazmatsVectorPtr->size());
    EXPECT_TRUE(true); 
  } 
  
  
 
  
int main(int argc, char **argv) {
  
  ros::Time::init();
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
  

  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  

