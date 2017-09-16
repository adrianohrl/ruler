/**
 *  This source file implements the ContinuousStepFunction class, which is based
 *on the StepFunction templated class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "utilities/functions/continuous_step_function.h"

namespace utilities
{
namespace functions
{
ContinuousStepFunction::ContinuousStepFunction(double qf, bool ascending,
                                               bool negated)
    : StepFunction<utilities::ContinuousSignalType>::StepFunction(
          0.0, qf, ascending, negated)
{
}
ContinuousStepFunction::ContinuousStepFunction(double d0, double qf,
                                               bool ascending, bool negated)
    : StepFunction<utilities::ContinuousSignalType>::StepFunction(
          d0, qf, ascending, negated)
{
}

ContinuousStepFunction::ContinuousStepFunction(const ros::Duration& d0,
                                               double qf, bool ascending,
                                               bool negated)
    : StepFunction<utilities::ContinuousSignalType>::StepFunction(
          d0, qf, ascending, negated)
{
}

ContinuousStepFunction::ContinuousStepFunction(double d0, double q0, double qf,
                                               bool ascending, bool negated)
    : StepFunction<utilities::ContinuousSignalType>::StepFunction(
          d0, q0, qf, ascending, negated)
{
}

ContinuousStepFunction::ContinuousStepFunction(const ros::Duration& d0,
                                               double q0, double qf,
                                               bool ascending, bool negated)
    : StepFunction<utilities::ContinuousSignalType>::StepFunction(
          d0, q0, qf, ascending, negated)
{
}

ContinuousStepFunction::ContinuousStepFunction(
    const ContinuousStepFunction& function)
    : StepFunction<utilities::ContinuousSignalType>::StepFunction(function)
{
}

ContinuousStepFunction::~ContinuousStepFunction() {}
}
}
