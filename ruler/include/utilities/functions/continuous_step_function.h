/**
 *  This header file defines the ContinuousStepFunction class, which is
 *based on the StepFunction templated class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_CONTINUOUS_STEP_FUNCTION_H_
#define _UTILITIES_CONTINUOUS_STEP_FUNCTION_H_

#include "utilities/continuous_signal_type.h"
#include "utilities/functions/step_function.h"

namespace utilities
{
namespace functions
{
class ContinuousStepFunction
    : public StepFunction<utilities::ContinuousSignalType>
{
public:
  ContinuousStepFunction(double qf, bool ascending = false);
  ContinuousStepFunction(double d0, double df, double q0, double qf,
                         bool ascending = false);
  ContinuousStepFunction(ros::Duration d0, ros::Duration df, double q0,
                         double qf, bool ascending = false);
  ContinuousStepFunction(const ContinuousStepFunction& function);
  virtual ~ContinuousStepFunction();
};
}
}

#endif // _UTILITIES_CONTINUOUS_STEP_FUNCTION_H_
