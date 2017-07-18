/**
 *  This source file implements the Exponential class, which is based on the
 *Function
 *abstract class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "utilities/exponential.h"

namespace utilities
{

Exponential::Exponential(double d0, double df, double q0, double qf,
                         bool ascending, double k, double base)
    : Function(d0, df, q0, qf, ascending), base_(base), k_(k)
{
}

Exponential::~Exponential() {}

double Exponential::calculate(double d) const
{
  double rate(k_ / (df_ - d0_));
  return qf_ - (qf_ - q0_) * pow(base_, rate * (d - d0_));
}
}
