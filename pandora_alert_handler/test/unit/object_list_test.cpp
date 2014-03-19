
// "Copyright [2014] <Pandora_Software_Testing_Team>" 
#include <cstdlib>
#include <ctime>
#include "alert_handler/object_list.h"
#include "gtest/gtest.h"

class ObjectListTest : public testing::Test 
{
 protected:
 
  /* Constructor/Destructor */

  ObjectListTest() : objectList(), objectList2(3, 10.669),
  object1(new Object), object2(new Object), object3(new Object),
  object4(new Object), object5(new Object), object6(new Object),
  object7(new Object), object8(new Object) {}

  /* SetUp/TearDown definitions */

  virtual void SetUp() 
  { 
    std::srand(std::time(0));
    
    seed = time(NULL);
    pose1.position.x = -0.5;
    pose1.position.y = 0;
    pose1.position.z = 0;      
    object1->setPose(pose1);
    object1->setId(1);
    
    pose2.position.x = 0;
    pose2.position.y = 0;
    pose2.position.z = 0;      
    object2->setPose(pose2);
    object2->setId(2);
    
    
    pose3.position.x = 0.5;
    pose3.position.y = 0;
    pose3.position.z = 0;      
    object3->setPose(pose3);
    object3->setId(3);

    pose4.position.x = -1;
    pose4.position.y = 0;
    pose4.position.z = 0;      
    object4->setPose(pose4);
    object4->setId(4);
    
    
    pose5.position.x = -0.125;
    pose5.position.y = -0.125;
    pose5.position.z = 0;      
    object5->setPose(pose5);
    object5->setId(5);
    
    pose6.position.x = 0;
    pose6.position.y = 0.4;
    pose6.position.z = 0;      
    object6->setPose(pose6);
    object6->setId(6);
    
    pose7.position.x = static_cast<double>
     (rand_r(&seed) - RAND_MAX/2)/(RAND_MAX/2) * 10000;
    pose7.position.y = 
      static_cast<double> (rand_r(&seed) - RAND_MAX/2)/(RAND_MAX/2) * 10000;
    pose7.position.z = 
      static_cast<double> (rand_r(&seed) - RAND_MAX/2)/(RAND_MAX/2) * 10000;      
    object7->setPose(pose7);
    object7->setId(7);
    
    
    pose8.position.x = 0.125;
    pose8.position.y = 0.125;
    pose8.position.z = 0;      
    object8->setPose(pose8);
    object8->setId(8);
  }

  /* Function to fill objectList.objects_ */

  void fillList(ObjectList<Object>* objList) 
  {
    getObjects(objList).clear();
    getObjects(objList).push_back(object1);
    getObjects(objList).push_back(object2);
    getObjects(objList).push_back(object3);
  }

  void fillListRandom(ObjectList<Object>* objList, int n) 
  {
    getObjects(objList).clear();

    for(int i = 0; i < n; ++i) 
    {
      geometry_msgs::Pose pose;
      pose.position.x =
        static_cast<double> (rand_r(&seed) - RAND_MAX/2)/(RAND_MAX/2) * 10000;
      pose.position.y=
       static_cast<double> (rand_r(&seed) - RAND_MAX/2)/(RAND_MAX/2) * 10000;
      pose.position.z =
        static_cast<double> (rand_r(&seed) - RAND_MAX/2)/(RAND_MAX/2) * 10000;   
      ObjectPtr object(new Object);
      object->setPose(pose);
      getObjects(objList).push_back(object);
    }
  }

  /* Accessors for private methods/members of ObjectList */

  bool isAnExistingObject(
      ObjectList<Object>* objList, const ObjectConstPtr& object, 
      ObjectList<Object>::IteratorList* iteratorListPtr) 
  {
    return objList->isAnExistingObject(object, iteratorListPtr);
  }

  void updateObject(
      ObjectList<Object>* objList, const ObjectPtr& object, 
      const ObjectList<Object>::IteratorList& iteratorList) 
  {
    objList->updateObject(object, iteratorList);
  }

  int* id(ObjectList<Object>* objList) 
  {
    return &(objList->id_);
  }

  ObjectList<Object>::List& getObjects(ObjectList<Object>* objList) 
  {
    return objList->objects_;
  }
  
  // get and set the params

  float* DIST_THRESHOLD(ObjectList<Object>* objList) 
  {
    return &(objList->DIST_THRESHOLD);
  }

  int* COUNTER_THRES(ObjectList<Object>* objList) 
  {
    return &(objList->COUNTER_THRES);
  }

  /* Variables */
  unsigned int  seed;
  ObjectList<Object> objectList;
  ObjectList<Object> objectList2;
  ObjectPtr object1;
  ObjectPtr object2;
  ObjectPtr object3;
  ObjectPtr object4;
  ObjectPtr object5;
  ObjectPtr object6;
  ObjectPtr object7;
  ObjectPtr object8;
  geometry_msgs::Pose pose1;
  geometry_msgs::Pose pose2;
  geometry_msgs::Pose pose3;
  geometry_msgs::Pose pose4;
  geometry_msgs::Pose pose5;
  geometry_msgs::Pose pose6;
  geometry_msgs::Pose pose7;
  geometry_msgs::Pose pose8;

};

TEST_F(ObjectListTest, Constructor) 
{
  EXPECT_EQ( 0u , objectList.size() );
  EXPECT_EQ( 0u , *id(&objectList) );
  EXPECT_EQ( 1u , *COUNTER_THRES(&objectList) );
  EXPECT_NEAR( 0.5 , *DIST_THRESHOLD(&objectList) , 0.0001 );

  EXPECT_EQ( 0u , objectList2.size() );
  EXPECT_EQ( 0u , *id(&objectList2) );
  EXPECT_EQ( 3 , *COUNTER_THRES(&objectList2) );
  EXPECT_NEAR( 10.669 , *DIST_THRESHOLD(&objectList2) , 0.0001 );
}

TEST_F(ObjectListTest, IsAnExistingObject) 
{  
  fillList(&objectList);
  ObjectList<Object>::IteratorList iteratorList;
  ObjectList<Object>::IteratorList::const_iterator it;
  
  
  ASSERT_EQ( 0.5, *DIST_THRESHOLD(&objectList) );

  // It shouldn't find that Object4 already exists.
  // Object4 won't correlate with Object1 because dist equals DIST_THRES!

  EXPECT_FALSE( isAnExistingObject(&objectList, object4, &iteratorList) );
  EXPECT_TRUE( iteratorList.empty() );
  iteratorList.clear();

  // It should find that newObject2 exists in 2 places.

  EXPECT_TRUE( isAnExistingObject(&objectList, object5, &iteratorList) );
  it = iteratorList.begin();
  EXPECT_FALSE( iteratorList.empty() );
  ASSERT_EQ( 2u , iteratorList.size() );
  EXPECT_EQ( object1, **(it++) );
  EXPECT_EQ( object2, **(it) );

  iteratorList.clear();

  // It should find that object6 exists only  in 1 place( same as object 3).
  EXPECT_TRUE( iteratorList.empty() );
  EXPECT_TRUE( isAnExistingObject(&objectList, object6, &iteratorList) );
  it = iteratorList.begin();
  EXPECT_FALSE( iteratorList.empty() );
  ASSERT_EQ( 1u , iteratorList.size() );
  EXPECT_EQ( object2 , **it );

  iteratorList.clear();

  // Changed distance must find Object 6 in  3 places( object1 object2 object3).
  (*DIST_THRESHOLD(&objectList)) = 1; 
  EXPECT_EQ( 1u , *DIST_THRESHOLD(&objectList) );
  EXPECT_TRUE( isAnExistingObject(&objectList, object6, &iteratorList) );
  it = iteratorList.begin();
  EXPECT_FALSE( iteratorList.empty() );
  ASSERT_EQ( 3u , iteratorList.size() );
  EXPECT_EQ( object1 , **(it++) );
  EXPECT_EQ( object2 , **(it++) );
  EXPECT_EQ( object3 , **it );

  iteratorList.clear();

  // Or it doesn't find it.
  (*DIST_THRESHOLD(&objectList)) = 0.2;

  EXPECT_FALSE( isAnExistingObject(&objectList , object6 , &iteratorList) );
  EXPECT_TRUE( iteratorList.empty() );
  EXPECT_EQ( 0u , iteratorList.size() );

  iteratorList.clear();

  // Zero Distance Threshold makes impossible same Object recognition.
  (*DIST_THRESHOLD(&objectList)) = 0;
  fillListRandom(&objectList, 10000);

  EXPECT_FALSE( isAnExistingObject(&objectList , object7 , &iteratorList) );
  EXPECT_TRUE( iteratorList.empty() );
  EXPECT_EQ( 0u, iteratorList.size() );

  iteratorList.clear();

  // Maximum (Infinite) Distance Threshold makes impossible Object distinction.
  (*DIST_THRESHOLD(&objectList)) = FLT_MAX;

  EXPECT_TRUE( isAnExistingObject(&objectList, object7 , &iteratorList) );
  EXPECT_FALSE( iteratorList.empty() );
  EXPECT_EQ( objectList.size() , iteratorList.size() );

  iteratorList.clear();
}


TEST_F(ObjectListTest, Add) 
{ 
  ObjectList<Object>::const_iterator_vers_ref it = getObjects(&objectList).begin();
  *(COUNTER_THRES(&objectList)) = 4;
  
  ASSERT_EQ( 0u, objectList.size() );
  EXPECT_TRUE( objectList.add(object1) );
  ASSERT_EQ( 1u, objectList.size() );
  it = getObjects(&objectList).begin();
  EXPECT_EQ( object1, *it );
  EXPECT_EQ(1u, object1->getCounter() );
  EXPECT_FALSE( object1->getLegit() );

  // Add (0.5, 0, 0)
  EXPECT_TRUE( objectList.add(object3) );
  ASSERT_EQ( 2u, objectList.size() );
  it = getObjects(&objectList).begin();
  EXPECT_EQ( object1 , *(it++) );
  EXPECT_EQ( object3 , *it );
  EXPECT_EQ( 1u, object3->getCounter() );
  EXPECT_FALSE( object3->getLegit() );

  // Add (-0.125,-0,125, 0), deletes (-0.5, 0, 0). 
  // object5 will replace object1
  EXPECT_FALSE( objectList.add(object5) );
  ASSERT_EQ( 2u, objectList.size() );
  it = getObjects(&objectList).begin();
  EXPECT_EQ( object3, *(it++) );
  EXPECT_EQ( object5, *it );
  EXPECT_EQ( 2u, object5->getCounter() );
  EXPECT_FALSE( object5->getLegit() );
    
  // Add (0.125, 0.125, 0), deletes (-0.125,-0,125, 0) and (0.5, 0, 0). 
  // object8 will replace both object5 and object3
  EXPECT_FALSE( objectList.add(object8) );
  ASSERT_EQ( 1u, objectList.size() );
  it = getObjects(&objectList).begin();
  EXPECT_EQ(object8, *it );
  EXPECT_EQ( 4u, object8->getCounter() );
  EXPECT_FALSE( object8->getLegit() );

  // Add (0, 0, 0), deletes (0.125, 0.125, 0).
  // object2 replaces object8
  EXPECT_FALSE( objectList.add(object2));
  ASSERT_EQ( 1u, objectList.size() );
  it = getObjects(&objectList).begin();
  EXPECT_EQ( object2, *it );
  EXPECT_EQ( 5u, object2->getCounter() );
  EXPECT_TRUE( object2->getLegit() );
}

