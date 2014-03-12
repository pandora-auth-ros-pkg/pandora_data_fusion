#include <cstdlib>
#include <ctime>

#include "alert_handler/object_list.h"
#include "gtest/gtest.h"

class ObjectListTest : public testing::Test {
	protected:

	/* Constructor/Destructor */

	ObjectListTest() : objectList(), objectList2(3, 10.669), object1(new Object), object2(new Object), object3(new Object),
						object4(new Object), object5(new Object), object6(new Object), object7(new Object), object8(new Object) {}

	/* SetUp/TearDown definitions */

	virtual void SetUp()
	{
		std::srand(std::time(0));
		
		
	    pose1.position.x=-0.5;
        pose1.position.y=0;
        pose1.position.z=0;	    
	    object1->setPose(pose1);
		object1->setId(1);
		
		pose2.position.x=0;
        pose2.position.y=0;
        pose2.position.z=0;	    
	    object2->setPose(pose2);
		object2->setId(2);
		
		
		pose3.position.x=0.5;
        pose3.position.y=0;
        pose3.position.z=0;	    
	    object3->setPose(pose3);
		object3->setId(3);

		pose4.position.x=-1;
        pose4.position.y=0;
        pose4.position.z=0;	    
	    object4->setPose(pose4);
		object4->setId(4);
		
		
		pose5.position.x=-0.125;
        pose5.position.y=-0.125;
        pose5.position.z=0;	    
	    object5->setPose(pose5);
		object5->setId(5);
		
		pose6.position.x=0;
        pose6.position.y=0.4;
        pose6.position.z=0;	    
	    object6->setPose(pose5);
		object6->setId(6);
		
		pose7.position.x=(double) (std::rand() - RAND_MAX/2)/(RAND_MAX/2) * 10000;
        pose7.position.y=(double) (std::rand() - RAND_MAX/2)/(RAND_MAX/2) * 10000;
        pose7.position.z=(double) (std::rand() - RAND_MAX/2)/(RAND_MAX/2) * 10000;	    
	    object7->setPose(pose7);
		object7->setId(7);
		
		
		pose8.position.x=0.125;
        pose8.position.y=0.125;
        pose8.position.z=0;	    
	    object8->setPose(pose8);
		object8->setId(8);
	}

	/* Function to fill objectList.objects_ */

	void fillList(ObjectList<Object> *objList)
	{
		getObjects(objList).clear();
		getObjects(objList).push_back(object1);
		getObjects(objList).push_back(object2);
		getObjects(objList).push_back(object3);
	}

	void fillListRandom(ObjectList<Object> *objList, int n) {
		getObjects(objList).clear();

		for(int i = 0; i < n; ++i) {
			
			geometry_msgs::Pose pose;
			pose.position.x=(double) (std::rand() - RAND_MAX/2)/(RAND_MAX/2) * 10000;
            pose.position.y=(double) (std::rand() - RAND_MAX/2)/(RAND_MAX/2) * 10000;
            pose.position.z=(double) (std::rand() - RAND_MAX/2)/(RAND_MAX/2) * 10000;	 
			ObjectPtr object(new Object);
			object->setPose(pose);
			getObjects(objList).push_back(object);
		}
      }

	/* Accessors for private methods/members of ObjectList */

	bool isAnExistingObject(ObjectList<Object> objList, ObjectPtr object, ObjectList<Object>::IteratorList* iteratorListPtr) {
		return objList.isAnExistingObject(object, iteratorListPtr);
	}

	void updateObject(ObjectList<Object> objList, ObjectPtr object, ObjectList<Object>::IteratorList iteratorList) {
		objList.updateObject(object, iteratorList);
	}

	 int& id(ObjectList<Object> objList) {
	     return objList.id_;
	    }

	ObjectList<Object>::List& getObjects(ObjectList<Object>* objList) {
		return objList->objects_;
	}

	// params

	float& DIST_THRESHOLD(ObjectList<Object> objList) {
		return objList.DIST_THRESHOLD;
	}

	int& COUNTER_THRES(ObjectList<Object> objList) {
		return objList.COUNTER_THRES;
	}

	/* Variables */

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

TEST_F(ObjectListTest, Constructor) {
	EXPECT_EQ(0u, objectList.size());
	EXPECT_EQ(0u, id(objectList));
	EXPECT_EQ(1, COUNTER_THRES(objectList));
	EXPECT_EQ(0.5, DIST_THRESHOLD(objectList));

	EXPECT_EQ(0u, objectList2.size());
	EXPECT_EQ(0u, id(objectList2));
	EXPECT_EQ(3, COUNTER_THRES(objectList2));
	EXPECT_EQ(10669, int(DIST_THRESHOLD(objectList2)*1000));
}

TEST_F(ObjectListTest, IsAnExistingObject) {
	
	fillList(&objectList);
	ObjectList<Object>::IteratorList iteratorList;
	ObjectList<Object>::iterator it;
	
	
	ASSERT_EQ(0.5, DIST_THRESHOLD(objectList));

	// It shouldn't find that newObject1 already exists.
	// newObject1 won't correlate with sameObject1 because dist equals DIST_THRES!

	EXPECT_FALSE( isAnExistingObject(objectList, object4, &iteratorList));
	EXPECT_TRUE( iteratorList.empty());
	iteratorList.clear();

	// It should find that newObject2 exists in 2 places.

	EXPECT_TRUE(isAnExistingObject(objectList, object5, &iteratorList));
	it=*(iteratorList.begin());
	EXPECT_FALSE(iteratorList.empty());
	EXPECT_EQ(2, iteratorList.size());
	EXPECT_EQ(object1, *(it++));
	EXPECT_EQ(object2, *it);

	iteratorList.clear();

	// It should find that newObject3 exists in 1 place, only, because of distance.

	EXPECT_TRUE(isAnExistingObject(objectList, object6, &iteratorList));
	EXPECT_FALSE( iteratorList.empty());
	EXPECT_EQ(1,iteratorList.size());
	EXPECT_EQ(object2, *it);

	iteratorList.clear();

	// Changed distance finds newObject3 in 3 places.
	DIST_THRESHOLD(objectList) = 1;

	EXPECT_TRUE(isAnExistingObject(objectList, object6, &iteratorList));
	EXPECT_FALSE(iteratorList.empty());
	EXPECT_EQ(3, iteratorList.size());
	EXPECT_EQ(object1, *(it++));
	EXPECT_EQ(object2, *(it++));
	EXPECT_EQ(object3, *it);

	iteratorList.clear();

	// Or it doesn't find it.
	DIST_THRESHOLD(objectList) = 0.2;

	EXPECT_FALSE( isAnExistingObject(objectList, object6, &iteratorList));
	EXPECT_TRUE( iteratorList.empty());
	EXPECT_EQ(0, iteratorList.size());

	iteratorList.clear();

	// Zero Distance Threshold makes impossible same Object recognition.
	DIST_THRESHOLD(objectList) = 0;
	fillListRandom(&objectList, 10000);

	EXPECT_FALSE( isAnExistingObject(objectList, object7, &iteratorList));
	EXPECT_TRUE( iteratorList.empty());
	EXPECT_EQ(0, iteratorList.size());

	iteratorList.clear();

	// Maximum (Infinite) Distance Threshold makes impossible Object distinction.
	DIST_THRESHOLD(objectList) = -1;

	EXPECT_TRUE( isAnExistingObject(objectList, object7, &iteratorList));
	EXPECT_FALSE( iteratorList.empty());
	EXPECT_EQ(objectList.size(), iteratorList.size());

	iteratorList.clear();
}

//~ TEST_F(ObjectListTest, Add) {
    //~ COUNTER_THRES(objectList) = 4;
	//~ EXPECT_EQ(0u, objectList.size());

	// Add (-0.5, 0, 0)
	//~ EXPECT_EQ(true, objectList.add(object1));
	//~ EXPECT_EQ(1u, objectList.size());
	//~ EXPECT_EQ(true, objectList.find(object1));
	//~ EXPECT_EQ(1u, object1->counter);
	//~ EXPECT_EQ(false, object1->legit);

	// Add (0.5, 0, 0)
	//~ EXPECT_EQ(true, objectList.add(object3));
	//~ EXPECT_EQ(2u, objectList.size());
	//~ EXPECT_EQ(true, objectList.find(object1));
	//~ EXPECT_EQ(true, objectList.find(object3));
	//~ EXPECT_EQ(1u, object3->counter);
	//~ EXPECT_EQ(false, object3->legit);

	// Add (-0.125,-0,125, 0), deletes (-0.5, 0, 0)
	//~ EXPECT_EQ(false, objectList.add(object5));
	//~ EXPECT_EQ(2u, objectList.size());
	//~ EXPECT_EQ(true, objectList.find(object5));
	//~ EXPECT_EQ(false, objectList.find(object1));
	//~ EXPECT_EQ(true, objectList.find(object3));
	//~ EXPECT_EQ(2u, object5->counter);
	//~ EXPECT_EQ(false, object5->legit);

	// Add (0.125, 0.125, 0), deletes (-0.125,-0,125, 0) and (0.5, 0, 0)
	//~ EXPECT_EQ(true, objectList.add(object8));
	//~ EXPECT_EQ(1u, objectList.size());
	//~ EXPECT_EQ(true, objectList.find(object8));
	//~ EXPECT_EQ(false, objectList.find(object5));
	//~ EXPECT_EQ(false, objectList.find(object1));
	//~ EXPECT_EQ(false, objectList.find(object3));
	//~ EXPECT_EQ(4u, object8->counter);
	//~ EXPECT_EQ(false, object8->legit);

	// Add (0, 0, 0), deletes (0.125, 0.125, 0)
	//~ EXPECT_EQ(true, objectList.add(object2));
	//~ EXPECT_EQ(1u, objectList.size());
	//~ EXPECT_EQ(true, objectList.find(object2));
	//~ EXPECT_EQ(false, objectList.find(object8));
	//~ EXPECT_EQ(false, objectList.find(object5));
	//~ EXPECT_EQ(false, objectList.find(object1));
	//~ EXPECT_EQ(false, objectList.find(object3));
	//~ EXPECT_EQ(5u, object2->counter);
	//~ EXPECT_EQ(true, object2->legit);t);
//~ }
