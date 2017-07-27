/**
 *  This header file defines the ExponentialFunction class, which is based on
 *the Function abstract class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_EXPONENTIAL_FUNCTION_H_
#define _UTILITIES_EXPONENTIAL_FUNCTION_H_

#include <cmath>
#include "utilities/function.h"

namespace utilities
{
class ExponentialFunction : public Function
{
public:
  ExponentialFunction(double d0, double df, double q0, double qf,
                      bool ascending = false, double k = 5, double base = M_E);
  ExponentialFunction(const ExponentialFunction& function);
  virtual ~ExponentialFunction();

private:
  double base_;
  double k_;
  virtual double calculate(double d) const;
};
}

#endif // _UTILITIES_EXPONENTIAL_FUNCTION_H_
