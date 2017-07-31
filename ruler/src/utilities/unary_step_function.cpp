/**
 *  This source file implements the UnaryStepFunction class, which is based
 *on the templated StepFunction class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "utilities/unary_step_function.h"

namespace utilities
{

UnaryStepFunction::UnaryStepFunction(double d0, bool ascending)
    : StepFunction<utilities::UnarySignalType>::StepFunction(
          d0, INFINITY, false, true, ascending)
{
}

UnaryStepFunction::UnaryStepFunction(ros::Duration d0, bool ascending)
    : StepFunction<utilities::UnarySignalType>::StepFunction(
          d0.toSec(), INFINITY, false, true, ascending)
{
}

UnaryStepFunction::UnaryStepFunction(const UnaryStepFunction& function)
    : StepFunction<utilities::UnarySignalType>::StepFunction(function)
{
}

UnaryStepFunction::~UnaryStepFunction() {}
}
