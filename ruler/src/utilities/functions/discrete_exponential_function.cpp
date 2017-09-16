/**
 *  This source file implements the DiscreteExponentialFunction class, which
 *is based on the ExponentialFunction templated class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "utilities/functions/discrete_exponential_function.h"

namespace utilities
{
namespace functions
{
DiscreteExponentialFunction::DiscreteExponentialFunction(double d0, double df,
                                                         double q0, double qf,
                                                         double k, double base,
                                                         bool ascending,
                                                         bool negated)
    : ExponentialFunction<utilities::DiscreteSignalType>::ExponentialFunction(
          d0, df, q0, qf, k, base, ascending, negated)
{
}

DiscreteExponentialFunction::DiscreteExponentialFunction(
    const ros::Duration& d0, const ros::Duration& df, double q0, double qf, double k,
    double base, bool ascending, bool negated)
    : ExponentialFunction<utilities::DiscreteSignalType>::ExponentialFunction(
          d0, df, q0, qf, k, base, ascending, negated)
{
}

DiscreteExponentialFunction::DiscreteExponentialFunction(
    const DiscreteExponentialFunction& function)
    : ExponentialFunction<utilities::DiscreteSignalType>::ExponentialFunction(
          function)
{
}

DiscreteExponentialFunction::~DiscreteExponentialFunction() {}
}
}
