/**
 *  This source file implements the UnaryStepFunction class, which is based
 *on the templated StepFunction class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "utilities/functions/unary_step_function.h"

namespace utilities
{
namespace functions
{
UnaryStepFunction::UnaryStepFunction(double d0, bool ascending, bool negated)
    : StepFunction<utilities::UnarySignalType>::StepFunction(d0, true,
                                                             ascending, negated)
{
}

UnaryStepFunction::UnaryStepFunction(const ros::Duration& d0, bool ascending,
                                     bool negated)
    : StepFunction<utilities::UnarySignalType>::StepFunction(d0.toSec(), true,
                                                             ascending, negated)
{
}

UnaryStepFunction::UnaryStepFunction(const UnaryStepFunction& function)
    : StepFunction<utilities::UnarySignalType>::StepFunction(function)
{
}

UnaryStepFunction::~UnaryStepFunction() {}
}
}
