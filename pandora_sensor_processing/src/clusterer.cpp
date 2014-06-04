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

#include "sensor_processing/clusterer.h"

namespace pandora_sensor_processing
{

  Clusterer::
    Clusterer(int measurementSize, 
        int maxClusterMemory, unsigned int maxIterations) :
      measurementSize_(measurementSize),
      maxClusterMemory_(maxClusterMemory),
      maxIterations_(maxIterations),
      readyToCluster_(false),
      oldestMeasurement_(0) {}

  void Clusterer::
    renewDataSet(Eigen::MatrixXf newMeasurement)
    {
      if(newMeasurement.cols() != measurementSize_ ||
          newMeasurement.rows() != 4)
        throw std::range_error("New measurement has flawed size.");
      if(dataSet_.cols() != maxClusterMemory_ * measurementSize_)
      {
        dataSet_.conservativeResize(Eigen::NoChange, dataSet_.cols() + measurementSize_);
      }
      else
      {
        if(oldestMeasurement_ == 3)
          oldestMeasurement_ = 0;
      }
      dataSet_.block(0, oldestMeasurement_ * measurementSize_,
          4, measurementSize_) << newMeasurement;
      oldestMeasurement_++;
      readyToCluster_ = true;
    }

  bool Clusterer::
    cluster()
    {
      if(!readyToCluster_)
        throw std::logic_error("Clusterer is not ready to cluster. Needs new measurement.");

      //!< Implementation of 2-means clustering
      bool converged = false;
      chooseInitialClusterCenters();
      for(int ii = 0; ii < maxIterations_; ++ii)
      {
        cluster1_.resize(4, 0);
        cluster2_.resize(4, 0);
        for(int jj = 0; jj < dataSet_.cols(); ++jj)
        {
          if(dataSet_.col(jj).transpose() * mean1_ < 
              dataSet_.col(jj).transpose() * mean2_)
          {
            cluster1_.conservativeResize(Eigen::NoChange, cluster1_.cols() + 1);
            cluster1_.col(cluster1_.cols() - 1) << dataSet_.col(jj);
          }
          else
          {
            cluster2_.conservativeResize(Eigen::NoChange, cluster2_.cols() + 1);
            cluster2_.col(cluster2_.cols() - 1) << dataSet_.col(jj);
          }
        }
        converged = calculateMeans();
        if(converged)
          return true;
      }

      readyToCluster_ = false;
      return false;
    }

  void Clusterer::
    calculateCovariances()
    {
      int ps = 0;
      Eigen::MatrixXf cluster_centered;

      ps = cluster1_.cols();
      cluster_centered = cluster1_.colwise() - mean1_;
      covariance1_ = cluster_centered.transpose() * cluster_centered / (ps - 1);

      ps = cluster2_.cols();
      cluster_centered = cluster2_.colwise() - mean2_;
      covariance2_ = cluster_centered.transpose() * cluster_centered / (ps - 1);
    }

  /**
   * @details Finds the means of current clusters. Also comprares these with
   * the means before to check for convergence.
   */
  bool Clusterer::
    calculateMeans()
    {
      Eigen::Vector4f temp1 = cluster1_.rowwise().mean();
      Eigen::Vector4f temp2 = cluster2_.rowwise().mean();

      bool converged = (temp1 - mean1_).norm() < 0.01 && (temp2 - mean2_).norm() < 0.01;

      mean1_ = temp1;
      mean2_ = temp2;

      return converged;
    }

  /**
   * @details Third row correspond to temperature. Picks as initial centroids
   * of clusters these data which have the lowest and the highest temperature.
   */
  void Clusterer::
    chooseInitialClusterCenters()
    {
      int minCol = 0, maxCol = 0;
      dataSet_.row(3).minCoeff(&minCol);
      dataSet_.row(3).maxCoeff(&maxCol);
      mean1_ = dataSet_.col(minCol);
      mean2_ = dataSet_.col(maxCol);
    }

}  // namespace pandora_sensor_processing
