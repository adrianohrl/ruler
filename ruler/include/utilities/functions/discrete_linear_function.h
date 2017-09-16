/**
 *  This header file defines the DiscreteLinearFunction class, which is
 *based on the LinearFunction templated class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_DISCRETE_LINEAR_FUNCTION_H_
#define _UTILITIES_DISCRETE_LINEAR_FUNCTION_H_

#include "utilities/discrete_signal_type.h"
#include "utilities/functions/linear_function.h"

namespace utilities
{
namespace functions
{
class DiscreteLinearFunction
    : public LinearFunction<utilities::DiscreteSignalType>
{
public:
  DiscreteLinearFunction(double d0, double df, double q0, double qf,
                         bool ascending = false, bool negated = false);
  DiscreteLinearFunction(const ros::Duration& d0, const ros::Duration& df,
                         double q0, double qf, bool ascending = false,
                         bool negated = false);
  DiscreteLinearFunction(const DiscreteLinearFunction& function);
  virtual ~DiscreteLinearFunction();
};

typedef boost::shared_ptr<DiscreteLinearFunction> DiscreteLinearFunctionPtr;
typedef boost::shared_ptr<DiscreteLinearFunction const>
    DiscreteLinearFunctionConstPtr;
}
}

#endif // _UTILITIES_DISCRETE_LINEAR_FUNCTION_H_
