// "Copyright [year] <Copyright Owner>"

#ifndef ALERT_HANDLER_FILTER_MODEL_H
#define ALERT_HANDLER_FILTER_MODEL_H

#include <vector>
#include <boost/shared_ptr.hpp>

#include <bfl/filter/extendedkalmanfilter.h>
#include <bfl/model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <bfl/model/linearanalyticmeasurementmodel_gaussianuncertainty.h>
#include <bfl/pdf/analyticconditionalgaussian.h>
#include <bfl/pdf/linearanalyticconditionalgaussian.h>

namespace pandora_data_fusion
{
namespace pandora_alert_handler
{

//!< Type Definitions
typedef BFL::LinearAnalyticConditionalGaussian
  AnalyticGaussian;
typedef boost::shared_ptr<AnalyticGaussian> AnalyticGaussianPtr;
typedef BFL::LinearAnalyticSystemModelGaussianUncertainty
  SystemModel;
typedef boost::shared_ptr<SystemModel> SystemModelPtr;
typedef std::vector<SystemModelPtr> SystemModelPtrVector;
typedef BFL::LinearAnalyticMeasurementModelGaussianUncertainty
  MeasurementModel;
typedef boost::shared_ptr<MeasurementModel> MeasurementModelPtr;
typedef std::vector<MeasurementModelPtr> MeasurementModelPtrVector;

class FilterModel
{
 public:

  FilterModel();

  SystemModelPtrVector getSystemModels() const;

  MeasurementModelPtrVector getMeasurementModels() const;

 private:
 
  //!< Filter's combined matrix
  std::vector<MatrixWrapper::Matrix> matrixAB_;
  //!< Filter's system pdf
  AnalyticGaussianPtr systemPdfPtr_;
  //!< Filter's system model for dimension x
  SystemModelPtr systemModelX_;
  //!< Filter's system model for dimension y
  SystemModelPtr systemModelY_;
  //!< Filter's system model for dimension z
  SystemModelPtr systemModelZ_;
  
  //!< Filter's measurement matrix H
  MatrixWrapper::Matrix matrixH_;
  //!< Filter's measurement pdf
  AnalyticGaussianPtr measurementPdfPtr_;
  //!< Filter's measurement model for dimension x
  MeasurementModelPtr measurementModelX_;
  //!< Filter's measurement model for dimension y
  MeasurementModelPtr measurementModelY_;
  //!< Filter's measurement model for dimension z
  MeasurementModelPtr measurementModelZ_;

};

}  // namespace pandora_alert_handler
}  // namespace pandora_data_fusion

#endif  // ALERT_HANDLER_FILTER_MODEL_H
