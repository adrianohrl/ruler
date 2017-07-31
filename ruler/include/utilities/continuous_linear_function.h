/**
 *  This header file defines the ContinuousLinearFunction class, which is
 *based on the LinearFunction templated class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_CONTINUOUS_LINEAR_FUNCTION_H_
#define _UTILITIES_CONTINUOUS_LINEAR_FUNCTION_H_

#include "utilities/continuous_signal_type.h"
#include "utilities/linear_function.h"

namespace utilities
{
class ContinuousLinearFunction
    : public LinearFunction<utilities::ContinuousSignalType>
{
public:
  ContinuousLinearFunction(double d0, double df, double q0, double qf,
                         bool ascending = false);
  ContinuousLinearFunction(ros::Duration d0, ros::Duration df, double q0,
                         double qf, bool ascending = false);
  ContinuousLinearFunction(const ContinuousLinearFunction& function);
  virtual ~ContinuousLinearFunction();
};
}

#endif // _UTILITIES_CONTINUOUS_LINEAR_FUNCTION_H_
