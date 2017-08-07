/**
 *  This source file implements the ContinuousPulseFunction class, which
 *is based on the PulseFunction templated class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "utilities/functions/continuous_pulse_function.h"

namespace utilities
{
namespace functions
{
ContinuousPulseFunction::ContinuousPulseFunction(
    const ContinuousStepFunction& step_function, double df)
    : PulseFunction<utilities::ContinuousSignalType>::PulseFunction(
          step_function, df)
{
}

ContinuousPulseFunction::ContinuousPulseFunction(
    const ContinuousStepFunction& step_function, ros::Duration df)
    : PulseFunction<utilities::ContinuousSignalType>::PulseFunction(
          step_function, df)
{
}

ContinuousPulseFunction::ContinuousPulseFunction(double d0, double df,
                                                 double qf, bool ascending,
                                                 bool negated)
    : PulseFunction<utilities::ContinuousSignalType>::PulseFunction(
          d0, df, qf, ascending, negated)
{
}

ContinuousPulseFunction::ContinuousPulseFunction(ros::Duration d0,
                                                 ros::Duration df, double qf,
                                                 bool ascending, bool negated)
    : PulseFunction<utilities::ContinuousSignalType>::PulseFunction(
          d0, df, qf, ascending, negated)
{
}

ContinuousPulseFunction::ContinuousPulseFunction(double d0, double df,
                                                 double q0, double qf,
                                                 bool ascending, bool negated)
    : PulseFunction<utilities::ContinuousSignalType>::PulseFunction(
          d0, df, q0, qf, ascending, negated)
{
}

ContinuousPulseFunction::ContinuousPulseFunction(ros::Duration d0,
                                                 ros::Duration df, double q0,
                                                 double qf, bool ascending,
                                                 bool negated)
    : PulseFunction<utilities::ContinuousSignalType>::PulseFunction(
          d0, df, q0, qf, ascending, negated)
{
}

ContinuousPulseFunction::ContinuousPulseFunction(
    const ContinuousPulseFunction& function)
    : PulseFunction<utilities::ContinuousSignalType>::PulseFunction(function)
{
}

ContinuousPulseFunction::~ContinuousPulseFunction() {}
}
}
