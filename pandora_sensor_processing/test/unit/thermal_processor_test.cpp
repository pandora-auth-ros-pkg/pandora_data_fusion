/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, P.A.N.D.O.R.A. Team.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the P.A.N.D.O.R.A. Team nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: 
 *   Tsirigotis Christos <tsirif@gmail.com>
 *********************************************************************/

#include "gtest/gtest.h"

#include "sensor_processing/thermal_processor.h"

using namespace Eigen;

namespace pandora_sensor_processing
{

  class ThermalProcessorTest : public ::testing::Test
  {
    public:

      /**
       * @brief constructor
       */
      ThermalProcessorTest() 
        : thermalProcessor_("Yo")
      {
        width_ = 2;
        height_ = 2;
        clusterer_.reset( new Clusterer(width_*height_, 3, 100) );
        image_.header.stamp = ros::Time::now();
        image_.header.frame_id = "Life_in_mono";
      }
        
    protected:

      /* helper functions */

      /**
       * @brief Makes an image with wrong dimensions.
       */
      void makeWrongImage()
      {
        image_.width = 8;
        image_.height = 2;
        for(int ii = 0; ii < image_.width * image_.height; ++ii)
          image_.data[ii] = ii;
      }

      /**
       * @brief Makes an image with the right dimensions that must
       * qualify to an alert.
       */
      void makeLegitImageWithAlert()
      {
        image_.width = width_;
        image_.height = height_;
        for(int ii = 0; ii < image_.width * image_.height; ++ii)
          image_.data[ii] = 25;
        image_.data[0] = 36;
      }

      /**
       * @brief Makes an image with the right dimensions that will
       * not qualify to an alert.
       */
      void makeLegitImageWithoutAlert()
      {
        image_.width = width_;
        image_.height = height_;
        for(int ii = 0; ii < image_.width * image_.height; ++ii)
          image_.data[ii] = 25;
      }

      /* accessors to private functions */

      bool analyzeImage(const sensor_msgs::Image& msg,
          const ClustererPtr& clusterer)
      {
        return thermalProcessor_.analyzeImage(msg, clusterer);
      }

      bool getResults(const sensor_msgs::Image& msg,
          const ClustererConstPtr& clusterer)
      {
        return thermalProcessor_.getResults(msg, clusterer);
      }

      /* variables */
      
      int width_;
      int height_;
      sensor_msgs::Image image_;
      ClustererPtr clusterer_;
      ThermalProcessor thermalProcessor_;
  };

  TEST_F(ThermalProcessorTest, analyzeImage_fail)
  {
    makeWrongImage();
    EXPECT_FALSE(analyzeImage(image_, clusterer_));
  }

  TEST_F(ThermalProcessorTest, analyzeImage_ok)
  {
    makeLegitImageWithAlert();
    EXPECT_EQ(0, clusterer_->getMeasurementsCounter());
    EXPECT_TRUE(analyzeImage(image_, clusterer_));
    EXPECT_EQ(1, clusterer_->getMeasurementsCounter());
  }

  TEST_F(ThermalProcessorTest, getResults_fail1)
  {
    makeLegitImageWithoutAlert();
    analyzeImage(image_, clusterer_);
    EXPECT_FALSE(getResults(image_, clusterer_));
  }

  TEST_F(ThermalProcessorTest, getResults_ok1)
  {
    makeLegitImageWithAlert();
    analyzeImage(image_, clusterer_);
    EXPECT_TRUE(getResults(image_, clusterer_));
    pandora_common_msgs::GeneralAlertMsg alert = thermalProcessor_.getAlert();
    EXPECT_EQ(image_.header.frame_id, alert.header.frame_id);
    EXPECT_EQ(image_.header.stamp, alert.header.stamp);
    EXPECT_GT(alert.probability, 0.4);
  }

  TEST_F(ThermalProcessorTest, getResults_fail2)
  {
    makeLegitImageWithAlert();
    analyzeImage(image_, clusterer_);
    makeLegitImageWithoutAlert();
    analyzeImage(image_, clusterer_);
    EXPECT_FALSE(getResults(image_, clusterer_));
  }

  TEST_F(ThermalProcessorTest, getResults_ok2)
  {
    makeLegitImageWithoutAlert();
    analyzeImage(image_, clusterer_);
    makeLegitImageWithAlert();
    analyzeImage(image_, clusterer_);
    EXPECT_TRUE(getResults(image_, clusterer_));
    pandora_common_msgs::GeneralAlertMsg alert = thermalProcessor_.getAlert();
    EXPECT_EQ(image_.header.frame_id, alert.header.frame_id);
    EXPECT_EQ(image_.header.stamp, alert.header.stamp);
    EXPECT_GT(alert.probability, 0.4);
  }

  TEST_F(ThermalProcessorTest, getResults_fail3)
  {
    makeLegitImageWithoutAlert();
    analyzeImage(image_, clusterer_);
    makeLegitImageWithAlert();
    analyzeImage(image_, clusterer_);
    makeLegitImageWithoutAlert();
    analyzeImage(image_, clusterer_);
    EXPECT_FALSE(getResults(image_, clusterer_));
  }

}  // namespace pandora_sensor_processing




    


