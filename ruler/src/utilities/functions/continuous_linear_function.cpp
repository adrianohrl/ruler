/**
 *  This source file implements the ContinuousLinearFunction class, which is
 *based
 *on the LinearFunction templated class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "utilities/functions/continuous_linear_function.h"

namespace utilities
{
namespace functions
{
ContinuousLinearFunction::ContinuousLinearFunction(double d0, double df,
                                                   double q0, double qf,
                                                   bool ascending, bool negated)
    : LinearFunction<utilities::ContinuousSignalType>::LinearFunction(
          d0, df, q0, qf, ascending, negated)
{
}

ContinuousLinearFunction::ContinuousLinearFunction(ros::Duration d0,
                                                   ros::Duration df, double q0,
                                                   double qf, bool ascending,
                                                   bool negated)
    : LinearFunction<utilities::ContinuousSignalType>::LinearFunction(
          d0, df, q0, qf, ascending, negated)
{
}

ContinuousLinearFunction::ContinuousLinearFunction(
    const ContinuousLinearFunction& function)
    : LinearFunction<utilities::ContinuousSignalType>::LinearFunction(function)
{
}

ContinuousLinearFunction::~ContinuousLinearFunction() {}
}
}
