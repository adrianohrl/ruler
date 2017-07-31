/**
 *  This header file defines the UnaryStepFunction class, which is
 *based on the StepFunction templated class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_UNARY_STEP_FUNCTION_H_
#define _UTILITIES_UNARY_STEP_FUNCTION_H_

#include "utilities/unary_signal_type.h"
#include "utilities/step_function.h"

namespace utilities
{
class UnaryStepFunction : public StepFunction<utilities::UnarySignalType>
{
public:
  UnaryStepFunction(double d0 = 0.0, bool ascending = false);
  UnaryStepFunction(ros::Duration d0, bool ascending = false);
  UnaryStepFunction(const UnaryStepFunction& function);
  virtual ~UnaryStepFunction();
};
}

#endif // _UTILITIES_UNARY_STEP_FUNCTION_H_
