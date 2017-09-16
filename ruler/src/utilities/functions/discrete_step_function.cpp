/**
 *  This source file implements the DiscreteStepFunction class, which is based
 *on the templated StepFunction class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "utilities/functions/discrete_step_function.h"

namespace utilities
{
namespace functions
{
DiscreteStepFunction::DiscreteStepFunction(double qf, bool ascending,
                                           bool negated)
    : StepFunction<utilities::DiscreteSignalType>::StepFunction(
          0.0, qf, ascending, negated)
{
}

DiscreteStepFunction::DiscreteStepFunction(double d0, double qf, bool ascending,
                                           bool negated)
    : StepFunction<utilities::DiscreteSignalType>::StepFunction(
          d0, qf, ascending, negated)
{
}

DiscreteStepFunction::DiscreteStepFunction(const ros::Duration& d0, double qf,
                                           bool ascending, bool negated)
    : StepFunction<utilities::DiscreteSignalType>::StepFunction(
          d0, qf, ascending, negated)
{
}

DiscreteStepFunction::DiscreteStepFunction(double d0, double q0, double qf,
                                           bool ascending, bool negated)
    : StepFunction<utilities::DiscreteSignalType>::StepFunction(
          d0, q0, qf, ascending, negated)
{
}

DiscreteStepFunction::DiscreteStepFunction(const ros::Duration& d0, double q0,
                                           double qf, bool ascending,
                                           bool negated)
    : StepFunction<utilities::DiscreteSignalType>::StepFunction(
          d0, q0, qf, ascending, negated)
{
}

DiscreteStepFunction::DiscreteStepFunction(const DiscreteStepFunction& function)
    : StepFunction<utilities::DiscreteSignalType>::StepFunction(function)
{
}

DiscreteStepFunction::~DiscreteStepFunction() {}
}
}
