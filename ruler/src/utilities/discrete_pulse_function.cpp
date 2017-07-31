/**
 *  This source file implements the DiscretePulseFunction class, which
 *is based on the PulseFunction templated class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "utilities/discrete_pulse_function.h"

namespace utilities
{

DiscretePulseFunction::DiscretePulseFunction(
    double d0, double df, double q0, double qf, bool ascending)
    : PulseFunction<utilities::DiscreteSignalType>::PulseFunction(
          d0, df, q0, qf, ascending)
{
}

DiscretePulseFunction::DiscretePulseFunction(
    ros::Duration d0, ros::Duration df, double q0, double qf, bool ascending)
    : PulseFunction<utilities::DiscreteSignalType>::PulseFunction(
          d0, df, q0, qf, ascending)
{
}

DiscretePulseFunction::DiscretePulseFunction(
    const DiscretePulseFunction& function)
    : PulseFunction<utilities::DiscreteSignalType>::PulseFunction(
          function)
{
}

DiscretePulseFunction::~DiscretePulseFunction() {}
}
