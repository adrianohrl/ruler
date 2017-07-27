/**
 *  This source file implements the ExponentialFunction class, which is based on
 *the Function abstract class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "utilities/exponential_function.h"

namespace utilities
{
ExponentialFunction::ExponentialFunction(double d0, double df, double q0,
                                         double qf, bool ascending, double k,
                                         double base)
    : Function::Function(d0, df, q0, qf, ascending), base_(base), k_(fabs(k))
{
}

ExponentialFunction::ExponentialFunction(const ExponentialFunction& function)
    : Function::Function(function)
{
}

ExponentialFunction::~ExponentialFunction() {}

double ExponentialFunction::calculate(double d) const
{
  double rate(-k_ / (df_ - d0_));
  return qf_ - (qf_ - q0_) * pow(base_, rate * (d - d0_));
}
}
