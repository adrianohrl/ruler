/**
 *  This source file implements the StepFunction class, which is based on the
 *Function abstract class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "utilities/step_function.h"

namespace utilities
{
StepFunction::StepFunction(double d0, double df, double q0, double qf,
                           bool ascending)
    : Function::Function(d0, df, q0, qf, ascending)
{
}

StepFunction::StepFunction(const StepFunction& function)
    : Function::Function(function)
{
}

StepFunction::~StepFunction() {}

double StepFunction::calculate(double d) const { return qf_; }
}
