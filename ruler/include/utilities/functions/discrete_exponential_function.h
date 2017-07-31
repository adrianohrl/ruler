/**
 *  This header file defines the DiscreteExponentialFunction class, which is
 *based on the ExponentialFunction templated class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_DISCRETE_EXPONENTIAL_FUNCTION_H_
#define _UTILITIES_DISCRETE_EXPONENTIAL_FUNCTION_H_

#include "utilities/discrete_signal_type.h"
#include "utilities/functions/exponential_function.h"

namespace utilities
{
namespace functions
{
class DiscreteExponentialFunction
    : public ExponentialFunction<utilities::DiscreteSignalType>
{
public:
  DiscreteExponentialFunction(double d0, double df, double q0, double qf,
                              bool ascending = false);
  DiscreteExponentialFunction(ros::Duration d0, ros::Duration df, double q0,
                              double qf, bool ascending = false);
  DiscreteExponentialFunction(const DiscreteExponentialFunction& function);
  virtual ~DiscreteExponentialFunction();
};
}
}

#endif // _UTILITIES_DISCRETE_EXPONENTIAL_FUNCTION_H_
