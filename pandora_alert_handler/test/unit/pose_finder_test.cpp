// "Copyright 2014 <Tsirigotis Christos>"

#include <ros/ros.h>
#include <ros/package.h>

#include "alert_handler/pose_finder.h"
#include "map_loader/map_loader.h"
#include "gtest/gtest.h"

/**
@class PoseFinderTest
@brief Basic Test Fixture for testing PoseFinder
**/
class PoseFinderTest : public ::testing::Test {
 
 protected:

  /* Accessors for private methods of PoseFinder */
  Point
    positionOnWall(Point startPoint, float angle) const {

      return poseFinder_->positionOnWall(startPoint, angle);

  }

  float
    calcHeight(float alertPitch, float height, float distFromAlert) const {

      return poseFinder_->calcHeight(alertPitch, height, distFromAlert);

  }

  geometry_msgs::Quaternion 
    findNormalVectorOnWall(Point framePoint, Point alertPoint) const {

      return poseFinder_->findNormalVectorOnWall(framePoint, alertPoint);

  }

  std::pair<Point, Point>
    findDiameterEndPointsOnWall(std::vector<Point> points) const {

      return poseFinder_->findDiameterEndPointsOnWall(points);

  }

  int
    getOrientationCircle() const {

      return poseFinder_->ORIENTATION_CIRCLE;

    }

  int
    getOrientationDist() const {

      return poseFinder_->ORIENTATION_DIST;

    }

  float
    getApproachDist() const {

      return poseFinder_->APPROACH_DIST;

    }

  float
    getHeightHighThres() const {

      return poseFinder_->HEIGHT_HIGH_THRES;

    }

  float
    getHeightLowThres() const {

      return poseFinder_->HEIGHT_LOW_THRES;

    }

  float
    getOccupiedCellThres() const {

      return poseFinder_->OCCUPIED_CELL_THRES;

    }

  /* Methods */
  /**
  @brief Constructor
  **/
  PoseFinderTest() {

    ros::Time::init();

    map_.reset( new Map );
    *map_ = map_loader::loadMap(
      ros::package::getPath("pandora_alert_handler")
        +"/test/test_maps/map1.yaml");

  }

  /**
  @brief Function to SetUp the Test Fixture
  @return void
  **/
  virtual void
    SetUp() {
      
      tf::Matrix3x3 defaultRotation(tf::tfScalar(1), tf::tfScalar(0), tf::tfScalar(0),
                                    tf::tfScalar(0), tf::tfScalar(1), tf::tfScalar(0),
                                    tf::tfScalar(0), tf::tfScalar(0), tf::tfScalar(1));
      tf::Vector3 defaultTranslation(tf::tfScalar(0), tf::tfScalar(0), tf::tfScalar(0));
      defaultTransform_.setBasis(defaultRotation);
      defaultTransform_.setOrigin(defaultTranslation);

      poseFinder_.reset( new PoseFinder(map_, "TEST") );
    
  }

  /* Variables */
  MapPtr map_;

  tf::Transform defaultTransform_;

  PoseFinderPtr poseFinder_;
 
};

TEST_F(PoseFinderTest, updateParamsTest) {

  // Expect default parameters
  EXPECT_NEAR( 0.5 , getOccupiedCellThres() , 0.0001 );
  EXPECT_NEAR( 1.2 , getHeightHighThres() , 0.0001 );
  EXPECT_NEAR( 0 , getHeightLowThres() , 0.0001 );
  EXPECT_NEAR( 0.5 , getApproachDist() , 0.0001 );
  EXPECT_NEAR( 20 , getOrientationDist() , 0.0001 );
  EXPECT_NEAR( 10 , getOrientationCircle() , 0.0001 );

  poseFinder_->updateParams(0.6, 1.5, 0.3, 0.6, 30, 15);
  // Expect updated parameters
  EXPECT_NEAR( 0.6 , getOccupiedCellThres() , 0.0001 );
  EXPECT_NEAR( 1.5 , getHeightHighThres() , 0.0001 );
  EXPECT_NEAR( 0.3 , getHeightLowThres() , 0.0001 );
  EXPECT_NEAR( 0.6 , getApproachDist() , 0.0001 );
  EXPECT_NEAR( 30 , getOrientationDist() , 0.0001 );
  EXPECT_NEAR( 15 , getOrientationCircle() , 0.0001 );

}

TEST_F(PoseFinderTest, findAlertPoseTest) {

  poseFinder_->updateParams(0.5, 1.5, 0, 0.5, 20, 10);

  // Make a tfTransform [tf::Transform], check for various yaw [float] and pitches [float]
  // for the expected Pose

  float alertYaw, alertpitch;
  tf::Transform transform;
  Point position;
  tf::Quaternion orientation;
  orientation.x = 0;
  orientation.y = 0;
  Pose expected, result;

  alertYaw = 0.78540;
  alertPitch = 0.52360;
  transform = defaultTransform_;
  transform.setOrigin(tf::Vector3(tf::tfScalar(5),tf::tfScalar(5),tf::tfScalar(0.3)));
  position.x = 6.12;
  position y = 6.12;
  position.z = 0.87735;
  orientation.z = -0.70711;
  orientation.w = 0.70711;
  expected.position = position;
  expected.orientation = orientation;
  result = poseFinder_->findAlertPose(alertYaw, alertPitch, transform);
  EXPECT_NEAR( expected.position.x , result.position.x , 0.0001 );
  EXPECT_NEAR( expected.position.y , result.position.y , 0.0001 );
  EXPECT_NEAR( expected.position.z , result.position.z , 0.0001 );
  EXPECT_NEAR( expected.orientantion.x , result.orientation.x , 0.0001 );
  EXPECT_NEAR( expected.orientantion.y , result.orientation.y , 0.0001 );
  EXPECT_NEAR( expected.orientantion.z , result.orientation.z , 0.0001 );
  EXPECT_NEAR( expected.orientantion.w , result.orientation.w , 0.0001 );

}

TEST_F(PoseFinderTest, lookupTransformFromWorldTest) {
 
  poseFinder_->updateParams(0.5, 1.5, 0, 0.5, 20, 10);

  // Mock it bitch
   
}

TEST_F(PoseFinderTest, positionOnWallTest) {
 
  poseFinder_->updateParams(0.5, 1.5, 0, 0.5, 20, 10);

  // With given map [OccupancyGrid], make points [Point] and test their supposed positions
  // on wall with various angles [float]

  Point startPoint, expected, result;
  startPoint.z = 0;
  expected.z = 0;
  float angle;

  startPoint.x = 5;
  startPoint.y = 5;
  angle = 0;
  expected.x = 5;
  expected.y = 6;
  result = positionOnWall(startPoint, angle);
  EXPECT_NEAR( expected.x , result.x , 0.0001 );
  EXPECT_NEAR( expected.y , result.y , 0.0001 );

}

TEST_F(PoseFinderTest, calcHeightTest) {
 
  poseFinder_->updateParams(0.5, 1.5, 0, 0.5, 20, 10);

  // Given the height [float] of the robot coord. frame origin, check for various distances
  // from wall (distFromAlert) [float] and pitches [float] the corresponding alert height on
  // wall

  float h = 0.3;
  EXPECT_THROW( calcHeight(1.04720,h,-1) , AlertException );
  EXPECT_NEAR   ( 0.30000 , calcHeight(1.04720,h,0) , 0.0001 );
  EXPECT_THROW( calcHeight(1.04720,h,1) , AlertException );
  EXPECT_THROW( calcHeight(1.04720,h,1.5) , AlertException );
  EXPECT_THROW( calcHeight(1.04720,h,2) , AlertException );

  EXPECT_THROW( calcHeight(0.52360,h,-1) , AlertException );
  EXPECT_NEAR   ( 0.30000 , calcHeight(0.52360,h,0) , 0.0001 );
  EXPECT_NEAR   ( 0.87735 , calcHeight(0.52360,h,1) , 0.0001 );
  EXPECT_NEAR   ( 1.16603 , calcHeight(0.52360,h,1.5) , 0.0001 );
  EXPECT_NEAR   ( 1.45470 , calcHeight(0.52360,h,2) , 0.0001 );

  EXPECT_THROW( calcHeight(-0.52360,h,-1) , AlertException );
  EXPECT_NEAR   ( 0.30000 , calcHeight(-0.52360,h,0) , 0.0001 );
  EXPECT_THROW( calcHeight(-0.52360,h,1) , AlertException );
  EXPECT_THROW( calcHeight(-0.52360,h,1.5) , AlertException );
  EXPECT_THROW( calcHeight(-0.52360,h,2) , AlertException );

  h = 0.5;
  EXPECT_THROW( calcHeight(-0.26180,h,-1) , AlertException );
  EXPECT_NEAR   ( 0.500000 , calcHeight(-0.26180,h,0) , 0.0001 );
  EXPECT_NEAR   ( 0.232051 , calcHeight(-0.26180,h,1) , 0.0001 );
  EXPECT_NEAR   ( 0.098076 , calcHeight(-0.26180,h,1.5) , 0.0001 );
  EXPECT_THROW( calcHeight(-0.26180,h,2) , AlertException );

}

TEST_F(PoseFinderTest, findNormalVectorOnWallTest) {
 
  poseFinder_->updateParams(0.5, 1.5, 0, 0.5, 1, 0.5);
  // Test if the returned normal vector on wall is right [geometry_msgs::Quaternion]
  // with the given map [OccupancyGrid] and various frame points [Point] and 
  // alert points [Point]

  tf::Quaternion result, expected;
  expected.x = 0;
  expected.y = 0;
  Point framePoint;
  framePoint.z = 0;
  Point alertPoint;
  alertPoint.z = 0;

  expected.z = -0.70711;
  expected.w = 0.70711;
  framePoint.x = 5;
  framePoint.y = 5;
  alertPoint.x = 5;
  alertPoint.y = 6;
  result = findNormalVectorOnWall(framePoint, alertPoint);
  EXPECT_NEAR( expected.x , result.x , 0.0001 );
  EXPECT_NEAR( expected.y , result.y , 0.0001 );
  EXPECT_NEAR( expected.z , result.z , 0.0001 );
  EXPECT_NEAR( expected.w , result.w , 0.0001 );

  expected.z = 1;
  expected.w = 0;
  framePoint.x = 7;
  framePoint.y = 1;
  alertPoint.x = 8;
  alertPoint.y = 1;
  result = findNormalVectorOnWall(framePoint, alertPoint);
  EXPECT_NEAR( expected.x , result.x , 0.0001 );
  EXPECT_NEAR( expected.y , result.y , 0.0001 );
  EXPECT_NEAR( expected.z , result.z , 0.0001 );
  EXPECT_NEAR( expected.w , result.w , 0.0001 );

  expected.z = 1;
  expected.w = 0;
  framePoint.x = 10.0;
  framePoint.y = 12.3;
  alertPoint.x = 11.2;
  alertPoint.y = 12.8;
  result = findNormalVectorOnWall(framePoint, alertPoint);
  EXPECT_NEAR( expected.x , result.x , 0.0001 );
  EXPECT_NEAR( expected.y , result.y , 0.0001 );
  EXPECT_NEAR( expected.z , result.z , 0.0001 );
  EXPECT_NEAR( expected.w , result.w , 0.0001 );


  expected.z = 0;
  expected.w = 1;
  framePoint.x = 12.0;
  framePoint.y = 11.0;
  alertPoint.x = 11.2;
  alertPoint.y = 12.8;
  result = findNormalVectorOnWall(framePoint, alertPoint);
  EXPECT_NEAR( expected.x , result.x , 0.0001 );
  EXPECT_NEAR( expected.y , result.y , 0.0001 );
  EXPECT_NEAR( expected.z , result.z , 0.0001 );
  EXPECT_NEAR( expected.w , result.w , 0.0001 );

  expected.z = -0.70711;
  expected.w = 0.70711;
  framePoint.x = 4.15;
  framePoint.y = 2.7;
  alertPoint.x = 3;
  alertPoint.y = 3.5;
  result = findNormalVectorOnWall(framePoint, alertPoint);
  EXPECT_NEAR( expected.x , result.x , 0.0001 );
  EXPECT_NEAR( expected.y , result.y , 0.0001 );
  EXPECT_NEAR( expected.z , result.z , 0.0001 );
  EXPECT_NEAR( expected.w , result.w , 0.0001 );

  poseFinder_->updateParams(0.5, 1.5, 0, 0.5, 2, 1);
  result = findNormalVectorOnWall(framePoint, alertPoint);
  EXPECT_NEAR( expected.x , result.x , 0.0001 );
  EXPECT_NEAR( expected.y , result.y , 0.0001 );
  EXPECT_NEAR( expected.z , result.z , 0.0001 );
  EXPECT_NEAR( expected.w , result.w , 0.0001 );

}

TEST_F(PoseFinderTest, findDiameterEndPointsOnWallTest) {
 
  poseFinder_->updateParams(0.5, 1.5, 0, 0.5, 20, 10);

  // Given various vectors of points [std::vector<Point>] test if the largest
  // distance between them id given by the two points [std::pair<Point, Point>] returned.

  std::vector<Point> points;
  std::pair<Point, Point> result;

  points.clear();
  EXPECT_THROW( findDiameterEndPointsOnWall(points), AlertException );

  Point first;
  first.x = 0;
  first.y = 0;
  first.z = 0;
  points.push_back(first);
  EXPECT_THROW( findDiameterEndPointsOnWall(points), AlertException );

  Point second;
  second.x = 1;
  second.y = 0;
  second.z = 0;
  points.push_back(second);
  result = findDiameterEndPointsOnWall(points);
  EXPECT_NEAR( first.x , result.first.x , 0.0001 );
  EXPECT_NEAR( first.y , result.first.y , 0.0001 );
  EXPECT_NEAR( first.z , result.first.z , 0.0001 );
  EXPECT_NEAR( second.x , result.second.x , 0.0001 );
  EXPECT_NEAR( second.y , result.second.y , 0.0001 );
  EXPECT_NEAR( second.z , result.second.z , 0.0001 );

  Point third;
  third.x = 0;
  third.y = 0;
  third.z = 0;
  points.push_back(third);
  result = findDiameterEndPointsOnWall(points);
  EXPECT_NEAR( first.x , result.first.x , 0.0001 );
  EXPECT_NEAR( first.y , result.first.y , 0.0001 );
  EXPECT_NEAR( first.z , result.first.z , 0.0001 );
  EXPECT_NEAR( second.x , result.second.x , 0.0001 );
  EXPECT_NEAR( second.y , result.second.y , 0.0001 );
  EXPECT_NEAR( second.z , result.second.z , 0.0001 );

  Point fourth;
  fourth.x = 0;
  fourth.y = -1;
  fourth.z = 0;
  points.push_back(fourth);
  result = findDiameterEndPointsOnWall(points);
  EXPECT_NEAR( second.x , result.first.x , 0.0001 );
  EXPECT_NEAR( second.y , result.first.y , 0.0001 );
  EXPECT_NEAR( second.z , result.first.z , 0.0001 );
  EXPECT_NEAR( fourth.x , result.second.x , 0.0001 );
  EXPECT_NEAR( fourth.y , result.second.y , 0.0001 );
  EXPECT_NEAR( fourth.z , result.second.z , 0.0001 );


  Point fifth;
  fifth.x = 41;
  fifth.y = 2.42;
  fifth.z = -12;
  points.push_back(fifth);
  result = findDiameterEndPointsOnWall(points);
  EXPECT_NEAR( fourth.x , result.first.x , 0.0001 );
  EXPECT_NEAR( fourth.y , result.first.y , 0.0001 );
  EXPECT_NEAR( fourth.z , result.first.z , 0.0001 );
  EXPECT_NEAR( fifth.x , result.second.x , 0.0001 );
  EXPECT_NEAR( fifth.y , result.second.y , 0.0001 );
  EXPECT_NEAR( fifth.z , result.second.z , 0.0001 );


  Point sixth;
  sixth.x = 10000;
  sixth.y = 10000;
  sixth.z = 10000;
  points.push_back(sixth);
  result = findDiameterEndPointsOnWall(points);
  EXPECT_NEAR( fourth.x , result.first.x , 0.0001 );
  EXPECT_NEAR( fourth.y , result.first.y , 0.0001 );
  EXPECT_NEAR( fourth.z , result.first.z , 0.0001 );
  EXPECT_NEAR( sixth.x , result.second.x , 0.0001 );
  EXPECT_NEAR( sixth.y , result.second.y , 0.0001 );
  EXPECT_NEAR( sixth.z , result.second.z , 0.0001 );

}

