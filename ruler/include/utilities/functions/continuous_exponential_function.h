/**
 *  This header file defines the ContinuousExponentialFunction class, which is
 *based on the ExponentialFunction templated class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_CONTINUOUS_EXPONENTIAL_FUNCTION_H_
#define _UTILITIES_CONTINUOUS_EXPONENTIAL_FUNCTION_H_

#include "utilities/continuous_signal_type.h"
#include "utilities/functions/exponential_function.h"

namespace utilities
{
namespace functions
{
class ContinuousExponentialFunction
    : public ExponentialFunction<utilities::ContinuousSignalType>
{
public:
  ContinuousExponentialFunction(double d0, double df, double q0, double qf,
                                bool ascending = false);
  ContinuousExponentialFunction(ros::Duration d0, ros::Duration df, double q0,
                                double qf, bool ascending = false);
  ContinuousExponentialFunction(const ContinuousExponentialFunction& function);
  virtual ~ContinuousExponentialFunction();
};
}
}

#endif // _UTILITIES_CONTINUOUS_EXPONENTIAL_FUNCTION_H_
