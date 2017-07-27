/**
 *  This header file defines the LinearFunction class, which is based on the
 *Function abstract class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_LINEAR_FUNCTION_H_
#define _UTILITIES_LINEAR_FUNCTION_H_

#include "utilities/function.h"

namespace utilities
{
class LinearFunction : public Function
{
public:
  LinearFunction(double d0, double df, double q0, double qf,
                 bool ascending = false);
  LinearFunction(const LinearFunction& function);
  virtual ~LinearFunction();

private:
  virtual double calculate(double d) const;
};
}

#endif // _UTILITIES_LINEAR_FUNCTION_H_
