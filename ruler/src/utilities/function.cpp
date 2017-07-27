/**
 *  This source file implements the Function abstract class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "utilities/function.h"

namespace utilities
{
Function::Function(double d0, double df, double q0, double qf, bool ascending)
    : d0_(d0), df_(df), q0_(q0), qf_(qf), ascending_(ascending)
{
}

Function::Function(const Function& function)
    : d0_(function.d0_), df_(function.df_), q0_(function.q0_),
      qf_(function.qf_), ascending_(function.ascending_)
{
}

Function::~Function() {}

double Function::getValue(double d) const
{
  double q;
  if (d < d0_)
  {
    q = q0_;
  }
  else if (d <= df_)
  {
    q = calculate(d);
    if (q < q0_)
    {
      q = q0_;
    }
    else if (q > qf_)
    {
      q = qf_;
    }
  }
  else
  {
    q = qf_;
  }
  return ascending_ ? q : qf_ - q + q0_;
}

void Function::setAscending(bool ascending) { ascending_ = ascending; }
}
