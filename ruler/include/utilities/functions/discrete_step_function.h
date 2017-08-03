/**
 *  This header file defines the DiscreteStepFunction class, which is
 *based on the StepFunction templated class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_DISCRETE_STEP_FUNCTION_H_
#define _UTILITIES_DISCRETE_STEP_FUNCTION_H_

#include "utilities/discrete_signal_type.h"
#include "utilities/functions/step_function.h"

namespace utilities
{
namespace functions
{
class DiscreteStepFunction : public StepFunction<utilities::DiscreteSignalType>
{
public:
  DiscreteStepFunction(double qf, bool ascending = false, bool negated = false);
  DiscreteStepFunction(double d0, double qf, bool ascending = false,
                       bool negated = false);
  DiscreteStepFunction(ros::Duration d0, double qf, bool ascending = false,
                       bool negated = false);
  DiscreteStepFunction(double d0, double q0, double qf, bool ascending = false,
                       bool negated = false);
  DiscreteStepFunction(ros::Duration d0, double q0, double qf,
                       bool ascending = false, bool negated = false);
  DiscreteStepFunction(const DiscreteStepFunction& function);
  virtual ~DiscreteStepFunction();
};
}
}

#endif // _UTILITIES_DISCRETE_STEP_FUNCTION_H_
