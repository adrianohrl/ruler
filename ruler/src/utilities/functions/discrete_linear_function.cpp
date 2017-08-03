/**
 *  This source file implements the DiscreteLinearFunction class, which is
 *based on the LinearFunction templated class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "utilities/functions/discrete_linear_function.h"

namespace utilities
{
namespace functions
{
DiscreteLinearFunction::DiscreteLinearFunction(double d0, double df, double q0,
                                               double qf, bool ascending,
                                               bool negated)
    : LinearFunction<utilities::DiscreteSignalType>::LinearFunction(
          d0, df, q0, qf, ascending, negated)
{
}

DiscreteLinearFunction::DiscreteLinearFunction(ros::Duration d0,
                                               ros::Duration df, double q0,
                                               double qf, bool ascending,
                                               bool negated)
    : LinearFunction<utilities::DiscreteSignalType>::LinearFunction(
          d0, df, q0, qf, ascending, negated)
{
}

DiscreteLinearFunction::DiscreteLinearFunction(
    const DiscreteLinearFunction& function)
    : LinearFunction<utilities::DiscreteSignalType>::LinearFunction(function)
{
}

DiscreteLinearFunction::~DiscreteLinearFunction() {}
}
}
