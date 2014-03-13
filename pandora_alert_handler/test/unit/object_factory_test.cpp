#include "alert_handler/object_factory.h"
#include "gtest/gtest.h"

namespace {


class ObjectFactoryTest : public ::testing::Test {
 protected:
  
  ObjectFactoryTest(): map_type1("TEST"){
   MapPtr1.reset(new Map);
   ObjectFactoryPtr1.reset(new ObjectFactory(MapConstPtr(MapPtr1),map_type1));
  }

  virtual void SetUp() {
	  
  }

  //~ 
   //~ MapPtr createSimpleMap()
  //~ {
	  //~ 
	  //~ 
   //~ }
  
  
  /* variables */
  MapPtr MapPtr1;
  const std::string map_type1;
  ObjectFactoryPtr ObjectFactoryPtr1;
  

};

// Tests that the Foo::Bar() method does Abc.
TEST_F(ObjectFactoryTest,makeHoles) {
	
	EXPECT_TRUE(true);
  
}


}  // namespace

 
