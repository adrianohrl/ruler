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
DiscreteStepFunction::DiscreteStepFunction(double qf, bool ascending)
    : StepFunction<utilities::DiscreteSignalType>::StepFunction(qf, ascending)
{
}

DiscreteStepFunction::DiscreteStepFunction(double d0, double df, double q0,
                                           double qf, bool ascending)
    : StepFunction<utilities::DiscreteSignalType>::StepFunction(d0, df, q0, qf,
                                                                ascending)
{
}

DiscreteStepFunction::DiscreteStepFunction(ros::Duration d0, ros::Duration df,
                                           double q0, double qf, bool ascending)
    : StepFunction<utilities::DiscreteSignalType>::StepFunction(d0, df, q0, qf,
                                                                ascending)
{
}

DiscreteStepFunction::DiscreteStepFunction(const DiscreteStepFunction& function)
    : StepFunction<utilities::DiscreteSignalType>::StepFunction(function)
{
}

DiscreteStepFunction::~DiscreteStepFunction() {}
}
}
