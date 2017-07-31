/**
 *  This header file defines the UnaryPulseFunction class, which is
 *based on the PulseFunction templated class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_UNARY_PULSE_FUNCTION_H_
#define _UTILITIES_UNARY_PULSE_FUNCTION_H_

#include "utilities/unary_signal_type.h"
#include "utilities/pulse_function.h"

namespace utilities
{
class UnaryPulseFunction : public PulseFunction<utilities::UnarySignalType>
{
public:
  UnaryPulseFunction(double d0, double df, bool ascending = false);
  UnaryPulseFunction(ros::Duration d0, ros::Duration df,
                     bool ascending = false);
  UnaryPulseFunction(const UnaryPulseFunction& function);
  virtual ~UnaryPulseFunction();
};
}

#endif // _UTILITIES_UNARY_PULSE_FUNCTION_H_
