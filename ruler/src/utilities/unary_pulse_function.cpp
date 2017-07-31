/**
 *  This source file implements the UnaryPulseFunction class, which
 *is based on the PulseFunction templated class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "utilities/unary_pulse_function.h"

namespace utilities
{

UnaryPulseFunction::UnaryPulseFunction(double d0, double df, bool ascending)
    : PulseFunction<utilities::UnarySignalType>::PulseFunction(d0, df, false,
                                                               true, ascending)
{
}

UnaryPulseFunction::UnaryPulseFunction(ros::Duration d0, ros::Duration df,
                                       bool ascending)
    : PulseFunction<utilities::UnarySignalType>::PulseFunction(
          d0.toSec(), df.toSec(), false, true, ascending)
{
}

UnaryPulseFunction::UnaryPulseFunction(const UnaryPulseFunction& function)
    : PulseFunction<utilities::UnarySignalType>::PulseFunction(function)
{
}

UnaryPulseFunction::~UnaryPulseFunction() {}
}
