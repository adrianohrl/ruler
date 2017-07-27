/**
 *  This header file defines the StepFunction class, which is based on the
 *Function abstract class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_STEP_FUNCTION_H_
#define _UTILITIES_STEP_FUNCTION_H_

#include "utilities/function.h"

namespace utilities
{
class StepFunction : public Function
{
public:
  StepFunction(double d0, double df, double q0, double qf,
               bool ascending = false);
  StepFunction(const StepFunction& function);
  virtual ~StepFunction();

private:
  virtual double calculate(double d) const;
};
}

#endif // _UTILITIES_STEP_FUNCTION_H_
