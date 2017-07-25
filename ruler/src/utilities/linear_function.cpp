/**
 *  This source file implements the LinearFunction class, which is based on the
 *Function abstract class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "utilities/linear_function.h"

namespace utilities
{
LinearFunction::LinearFunction(double d0, double df, double q0, double qf,
                               bool ascending)
    : Function(d0, df, q0, qf, ascending)
{
}

LinearFunction::~LinearFunction() {}

double LinearFunction::calculate(double d) const
{
  double rate((qf_ - q0_) / (df_ - d0_));
  return rate * (d - d0_) + q0_;
}
}
