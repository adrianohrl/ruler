/**
 *  This header file defines the DiscretePulseFunction class, which is
 *based on the PulseFunction templated class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_DISCRETE_PULSE_FUNCTION_H_
#define _UTILITIES_DISCRETE_PULSE_FUNCTION_H_

#include "utilities/functions/discrete_step_function.h"
#include "utilities/functions/pulse_function.h"

namespace utilities
{
namespace functions
{
class DiscretePulseFunction
    : public PulseFunction<utilities::DiscreteSignalType>
{
public:
  DiscretePulseFunction(const DiscreteStepFunction& step_function, double df);
  DiscretePulseFunction(const DiscreteStepFunction& step_function, ros::Duration df);
  DiscretePulseFunction(double d0, double df, double qf, bool ascending = false,
                        bool negated = false);
  DiscretePulseFunction(ros::Duration d0, ros::Duration df, double qf,
                        bool ascending = false, bool negated = false);
  DiscretePulseFunction(double d0, double df, double q0, double qf,
                        bool ascending = false, bool negated = false);
  DiscretePulseFunction(ros::Duration d0, ros::Duration df, double q0,
                        double qf, bool ascending = false,
                        bool negated = false);
  DiscretePulseFunction(const DiscretePulseFunction& function);
  virtual ~DiscretePulseFunction();
};
}
}

#endif // _UTILITIES_DISCRETE_PULSE_FUNCTION_H_
