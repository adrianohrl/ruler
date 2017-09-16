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
#include "utilities/functions/linear_function.h"

namespace utilities
{
namespace functions
{
class ContinuousLinearFunction
    : public LinearFunction<utilities::ContinuousSignalType>
{
public:
  ContinuousLinearFunction(double d0, double df, double q0, double qf,
                           bool ascending = false, bool negated = false);
  ContinuousLinearFunction(const ros::Duration& d0, const ros::Duration& df,
                           double q0, double qf, bool ascending = false,
                           bool negated = false);
  ContinuousLinearFunction(const ContinuousLinearFunction& function);
  virtual ~ContinuousLinearFunction();
};

typedef boost::shared_ptr<ContinuousLinearFunction> ContinuousLinearFunctionPtr;
typedef boost::shared_ptr<ContinuousLinearFunction const>
    ContinuousLinearFunctionConstPtr;
}
}

#endif // _UTILITIES_CONTINUOUS_LINEAR_FUNCTION_H_
