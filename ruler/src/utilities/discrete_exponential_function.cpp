/**
 *  This source file implements the DiscreteExponentialFunction class, which
 *is based on the ExponentialFunction templated class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "utilities/discrete_exponential_function.h"

namespace utilities
{

DiscreteExponentialFunction::DiscreteExponentialFunction(double d0, double df,
                                                         double q0, double qf,
                                                         bool ascending)
    : ExponentialFunction<utilities::DiscreteSignalType>::ExponentialFunction(
          d0, df, q0, qf, ascending)
{
}

DiscreteExponentialFunction::DiscreteExponentialFunction(ros::Duration d0,
                                                         ros::Duration df,
                                                         double q0, double qf,
                                                         bool ascending)
    : ExponentialFunction<utilities::DiscreteSignalType>::ExponentialFunction(
          d0, df, q0, qf, ascending)
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
