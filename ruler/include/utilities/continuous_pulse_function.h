/**
 *  This header file defines the ContinuousPulseFunction class, which is
 *based on the PulseFunction templated class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_CONTINUOUS_PULSE_FUNCTION_H_
#define _UTILITIES_CONTINUOUS_PULSE_FUNCTION_H_

#include "utilities/continuous_signal_type.h"
#include "utilities/pulse_function.h"

namespace utilities
{
class ContinuousPulseFunction
    : public PulseFunction<utilities::ContinuousSignalType>
{
public:
  ContinuousPulseFunction(double d0, double df, double q0, double qf,
                          bool ascending = false);
  ContinuousPulseFunction(ros::Duration d0, ros::Duration df, double q0,
                          double qf, bool ascending = false);
  ContinuousPulseFunction(const ContinuousPulseFunction& function);
  virtual ~ContinuousPulseFunction();
};
}

#endif // _UTILITIES_CONTINUOUS_PULSE_FUNCTION_H_
