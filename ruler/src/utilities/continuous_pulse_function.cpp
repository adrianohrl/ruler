/**
 *  This source file implements the ContinuousPulseFunction class, which
 *is based on the PulseFunction templated class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "utilities/continuous_pulse_function.h"

namespace utilities
{

ContinuousPulseFunction::ContinuousPulseFunction(
    double d0, double df, double q0, double qf, bool ascending)
    : PulseFunction<utilities::ContinuousSignalType>::PulseFunction(
          d0, df, q0, qf, ascending)
{
}

ContinuousPulseFunction::ContinuousPulseFunction(
    ros::Duration d0, ros::Duration df, double q0, double qf, bool ascending)
    : PulseFunction<utilities::ContinuousSignalType>::PulseFunction(
          d0, df, q0, qf, ascending)
{
}

ContinuousPulseFunction::ContinuousPulseFunction(
    const ContinuousPulseFunction& function)
    : PulseFunction<utilities::ContinuousSignalType>::PulseFunction(
          function)
{
}

ContinuousExponentialFunction::~ContinuousExponentialFunction() {}
}
