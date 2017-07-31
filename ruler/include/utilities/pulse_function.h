/**
 *  This header file defines and implements the PulseFunction class, which is
 *based on the Function abstract class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_PULSE_FUNCTION_H_
#define _UTILITIES_PULSE_FUNCTION_H_

#include "utilities/function.h"

namespace utilities
{
template <typename T> class PulseFunction : public Function<T>
{
public:
  virtual ~PulseFunction();

protected:
  PulseFunction(double d0, double df, double q0, double qf,
                bool ascending = false);
  PulseFunction(ros::Duration d0, ros::Duration df, double q0, double qf,
                bool ascending = false);
  PulseFunction(const PulseFunction<T>& function);

private:
  virtual double calculate(double d) const;
};

template <typename T>
PulseFunction<T>::PulseFunction(double d0, double df, double q0, double qf,
                                bool ascending)
    : Function<T>::Function(d0, df, q0, qf, ascending, false)
{
}

template <typename T>
PulseFunction<T>::PulseFunction(ros::Duration d0, ros::Duration df, double q0,
                                double qf, bool ascending)
    : Function<T>::Function(d0, df, q0, qf, ascending, false)
{
}

template <typename T>
PulseFunction<T>::PulseFunction(const PulseFunction<T>& function)
    : Function<T>::Function(function)
{
}

template <typename T> PulseFunction<T>::~PulseFunction() {}

template <typename T> double PulseFunction<T>::calculate(double d) const
{
  return d < Function<T>::d0_ || d > Function<T>::df_ ? Function<T>::q0_
                                                      : Function<T>::qf_;
}
}

#endif // _UTILITIES_PULSE_FUNCTION_H_
