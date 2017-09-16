/**
 *  This source file implements the UnaryPulseFunction class, which
 *is based on the PulseFunction templated class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "utilities/functions/unary_pulse_function.h"

namespace utilities
{
namespace functions
{
UnaryPulseFunction::UnaryPulseFunction(const UnaryStepFunction& step_function,
                                       double df)
    : PulseFunction<utilities::UnarySignalType>::PulseFunction(step_function,
                                                               df)
{
}

UnaryPulseFunction::UnaryPulseFunction(const UnaryStepFunction& step_function,
                                       ros::Duration df)
    : PulseFunction<utilities::UnarySignalType>::PulseFunction(step_function,
                                                               df)
{
}

UnaryPulseFunction::UnaryPulseFunction(double d0, double df, bool ascending,
                                       bool negated)
    : PulseFunction<utilities::UnarySignalType>::PulseFunction(
          d0, df, false, true, ascending, negated)
{
}

UnaryPulseFunction::UnaryPulseFunction(const ros::Duration& d0,
                                       const ros::Duration& df, bool ascending,
                                       bool negated)
    : PulseFunction<utilities::UnarySignalType>::PulseFunction(
          d0.toSec(), df.toSec(), false, true, ascending, negated)
{
}

UnaryPulseFunction::UnaryPulseFunction(const UnaryPulseFunction& function)
    : PulseFunction<utilities::UnarySignalType>::PulseFunction(function)
{
}

UnaryPulseFunction::~UnaryPulseFunction() {}
}
}
