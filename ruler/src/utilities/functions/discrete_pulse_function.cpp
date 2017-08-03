/**
 *  This source file implements the DiscretePulseFunction class, which
 *is based on the PulseFunction templated class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "utilities/functions/discrete_pulse_function.h"

namespace utilities
{
namespace functions
{
DiscretePulseFunction::DiscretePulseFunction(double d0, double df, double qf,
                                             bool ascending, bool negated)
    : PulseFunction<utilities::DiscreteSignalType>::PulseFunction(
          d0, df, qf, ascending, negated)
{
}

DiscretePulseFunction::DiscretePulseFunction(ros::Duration d0, ros::Duration df,
                                             double qf, bool ascending,
                                             bool negated)
    : PulseFunction<utilities::DiscreteSignalType>::PulseFunction(
          d0, df, qf, ascending, negated)
{
}

DiscretePulseFunction::DiscretePulseFunction(double d0, double df, double q0,
                                             double qf, bool ascending,
                                             bool negated)
    : PulseFunction<utilities::DiscreteSignalType>::PulseFunction(
          d0, df, q0, qf, ascending, negated)
{
}

DiscretePulseFunction::DiscretePulseFunction(ros::Duration d0, ros::Duration df,
                                             double q0, double qf,
                                             bool ascending, bool negated)
    : PulseFunction<utilities::DiscreteSignalType>::PulseFunction(
          d0, df, q0, qf, ascending, negated)
{
}

DiscretePulseFunction::DiscretePulseFunction(
    const DiscretePulseFunction& function)
    : PulseFunction<utilities::DiscreteSignalType>::PulseFunction(function)
{
}

DiscretePulseFunction::~DiscretePulseFunction() {}
}
}
