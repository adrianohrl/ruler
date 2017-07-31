/**
 *  This source file implements the ContinuousStepFunction class, which is based
 *on the StepFunction templated class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "utilities/continuous_step_function.h"

namespace utilities
{

ContinuousStepFunction::ContinuousStepFunction(double qf, bool ascending)
    : StepFunction<utilities::ContinuousSignalType>::StepFunction(qf, ascending)
{
}

ContinuousStepFunction::ContinuousStepFunction(double d0, double df, double q0,
                                               double qf, bool ascending)
    : StepFunction<utilities::ContinuousSignalType>::StepFunction(d0, df, q0,
                                                                  qf, ascending)
{
}

ContinuousStepFunction::ContinuousStepFunction(ros::Duration d0,
                                               ros::Duration df, double q0,
                                               double qf, bool ascending)
    : StepFunction<utilities::ContinuousSignalType>::StepFunction(d0, df, q0,
                                                                  qf, ascending)
{
}

ContinuousStepFunction::ContinuousStepFunction(
    const ContinuousStepFunction& function)
    : StepFunction<utilities::ContinuousSignalType>::StepFunction(function)
{
}

ContinuousStepFunction::~ContinuousStepFunction() {}
}
