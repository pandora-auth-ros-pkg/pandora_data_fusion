// "Copyright 2014 <Tsirigotis Christos>"

#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>

#include "alert_handler/pose_finder.h"
#include "gtest/gtest.h"

namespace PoseFinder {

/**
* @class PoseFinderTest
* @brief Basic Test Fixture for testing PoseFinder
**/
class PoseFinderTest : public ::testing::Test {
 
public:

  /**
    * @brief Callback of mapSuscriber_
    * @param msg [nav_msgs::OccupancyGridConstPtr const&] Msg contains map from tester.
    * @return void
    **/
  void
    updateMap(const nav_msgs::OccupancyGridConstPtr& msg) {

    map_ = msg;

  }

 protected:

  /* Accessors for private methods of PoseFinder */
  Point
    positionOnWall(Point startPoint, angle) const {

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
    * @brief Constructor
    **/
  PoseFinderTest() {

    map_.reset( new Map );
    mapSubsriber_ = nh_.subscribe("/map", 1, &PoseFinderTest::updateMap, this);
    ros::spinOnce();

  }

  /**
    * @brief Function to SetUp the Test Fixture
    * @return void
    **/
  virtual void
    SetUp() {

    poseFinder_.reset( new PoseFinder(map_, "TEST") );
    
  }

  /* Variables */
  ros::NodeHandle nh_;
  ros::Subscriber mapSubscriber_;
  MapPtr map_;

  PoseFinderPtr poseFinder_;

 
};

}

TEST_F(PoseFinderTest, updateParamsTest) {

  // Expect default parameters
  EXPECT_EQ( 0.5 , getOccupiedCellThres() );
  EXPECT_EQ( 1.2 , getHeightHighThres() );
  EXPECT_EQ( 0 , getHeightLowThres() );
  EXPECT_EQ( 0.5 , getApproachDist() );
  EXPECT_EQ( 20 , getOrientationDist() );
  EXPECT_EQ( 10 , getOrientationCircle() );

  poseFinder_->updateParams(0.6, 1.5, 0.3, 0.6, 30, 15);
  // Expect updated parameters
  EXPECT_EQ( 0.6 , getOccupiedCellThres() );
  EXPECT_EQ( 1.5 , getHeightHighThres() );
  EXPECT_EQ( 0.3 , getHeightLowThres() );
  EXPECT_EQ( 0.6 , getApproachDist() );
  EXPECT_EQ( 30 , getOrientationDist() );
  EXPECT_EQ( 15 , getOrientationCircle() );

}

TEST_F(PoseFinderTest, findAlertPoseTest) {

  poseFinder_->updateParams(0.5, 1.5, 0, 0.5, 20, 10);

  // Make a tfTransform [tf::Transform], check for various yaw [float] and pitches [float]

}

TEST_F(PoseFinderTest, lookupTransformFromWorldTest) {
 
  poseFinder_->updateParams(0.5, 1.5, 0, 0.5, 20, 10);

  // Mock it bitch
   
}

TEST_F(PoseFinderTest, positionOnWallTest) {
 
  poseFinder_->updateParams(0.5, 1.5, 0, 0.5, 20, 10);

  // With given map [OccupancyGrid], make points [Point] and test their supposed positions
  // on wall with various angles [float]
   
}

TEST_F(PoseFinderTest, calcHeightTest) {
 
  poseFinder_->updateParams(0.5, 1.5, 0, 0.5, 20, 10);

  // Given the height [float] of the robot coord. frame origin, check for various distances
  // from wall (distFromAlert) [float] and pitches [float] the corresponding alert height on
  // wall

  float h = 0.3;
  EXPECT_THROW( calcHeight(1.04720,h,-1) , AlertException );
  EXPECT_EQ   ( 0.30000 , calcHeight(1.04720,h,0) );
  EXPECT_THROW( calcHeight(1.04720,h,1) , AlertException );
  EXPECT_THROW( calcHeight(1.04720,h,1.5) , AlertException );
  EXPECT_THROW( calcHeight(1.04720,h,2) , AlertException );

  EXPECT_THROW( calcHeight(0.52360,h,-1) , AlertException );
  EXPECT_EQ   ( 0.30000 , calcHeight(0.52360,h,0) );
  EXPECT_EQ   ( 0.87735 , calcHeight(0.52360,h,1) );
  EXPECT_EQ   ( 1.16603 , calcHeight(0.52360,h,1.5) );
  EXPECT_EQ   ( 1.45470 , calcHeight(0.52360,h,2) );

  EXPECT_THROW( calcHeight(-0.52360,h,-1) , AlertException );
  EXPECT_EQ   ( 0.30000 , calcHeight(-0.52360,h,0) );
  EXPECT_THROW( calcHeight(-0.52360,h,1) , AlertException );
  EXPECT_THROW( calcHeight(-0.52360,h,1.5) , AlertException );
  EXPECT_THROW( calcHeight(-0.52360,h,2) , AlertException );

  h = 0.5;
  EXPECT_THROW( calcHeight(-0.26180,h,-1) , AlertException );
  EXPECT_EQ   ( 0.500000 , calcHeight(-0.26180,h,0) );
  EXPECT_EQ   ( 0.232051 , calcHeight(-0.26180,h,1) );
  EXPECT_EQ   ( 0.098076 , calcHeight(-0.26180,h,1.5) );
  EXPECT_THROW( calcHeight(-0.26180,h,2) , AlertException );

}

TEST_F(PoseFinderTest, findNormalVectorOnWallTest) {
 
  poseFinder_->updateParams(0.5, 1.5, 0, 0.5, 20, 10);

  // Test if the returned normal vector on wall is right [geometry_msgs::Quaternion]
  // with the given map [OccupancyGrid] and various frame points [Point] and 
  // alert points [Point]
   
}

TEST_F(PoseFinderTest, findDiameterEndPointsOnWallTest) {
 
  poseFinder_->updateParams(0.5, 1.5, 0, 0.5, 20, 10);

  // Given various vectors of points [std::vector<Point>] test if the largest
  // distance between them id given by the two points [std::pair<Point, Point>] returned.

  std::vector<Point> points;
  std::pair<Point, Point> result;
  points.clear();
  EXPECT_THROW( findDiameterEndPointsOnWall(points), AlertException );
  points.push_back(Point(0,0,0));
  EXPECT_THROW( findDiameterEndPointsOnWall(points), AlertException );
  points.push_back(Point(1,0,0));
  result = findDiameterEndPointsOnWall(points);
  EXPECT_EQ( Point(0,0,0) , result.first );
  EXPECT_EQ( Point(1,0,0) , result.second );
  points.push_back(Point(0,0,0));
  result = findDiameterEndPointsOnWall(points);
  EXPECT_EQ( Point(0,0,0) , result.first );
  EXPECT_EQ( Point(1,0,0) , result.second );
  points.push_back(Point(0,-1,0));
  result = findDiameterEndPointsOnWall(points);
  EXPECT_EQ( Point(1,0,0) , result.first );
  EXPECT_EQ( Point(0,-1,0) , result.second );
  points.push_back(Point(41,2.42,-12));
  result = findDiameterEndPointsOnWall(points);
  EXPECT_EQ( Point(0,-1,0) , result.first );
  EXPECT_EQ( Point(41,2.42,-12) , result.second );
  points.push_back(Point(10000,10000,10000));
  result = findDiameterEndPointsOnWall(points);
  EXPECT_EQ( Point(0,-1,0) , result.first );
  EXPECT_EQ( Point(10000,10000,10000) , result.second );
 
}

