/**
 *  This header file defines and implements the PulseFunction class, which is
 *based on the Function abstract class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_PULSE_FUNCTION_H_
#define _UTILITIES_PULSE_FUNCTION_H_

#include "utilities/functions/function.h"
#include "utilities/functions/step_function.h"

namespace utilities
{
namespace functions
{
template <typename T> class PulseFunction : public Function<T>
{
public:
  PulseFunction(double d0, double df, double qf, bool ascending, bool negated);
  PulseFunction(ros::Duration d0, ros::Duration df, double qf, bool ascending,
                bool negated);
  PulseFunction(const StepFunction<T>& step_function, double df);
  PulseFunction(const StepFunction<T>& step_function, ros::Duration df);
  virtual ~PulseFunction();

protected:
  PulseFunction(double d0, double df, double q0, double qf, bool ascending,
                bool negated);
  PulseFunction(ros::Duration d0, ros::Duration df, double q0, double qf,
                bool ascending, bool negated);
  PulseFunction(const PulseFunction<T>& function);

private:
  virtual double calculate(double d) const;
};

template <typename T>
PulseFunction<T>::PulseFunction(double d0, double df, double qf, bool ascending,
                                bool negated)
    : Function<T>::Function("Pulse", d0, df, 0.0, qf, ascending, false, negated)
{
  if (d0 == 0.0 || df >= INFINITY)
  {
    throw utilities::Exception(
        "Use the step function instead of the pulse one.");
  }
}

template <typename T>
PulseFunction<T>::PulseFunction(ros::Duration d0, ros::Duration df, double qf,
                                bool ascending, bool negated)
    : Function<T>::Function("Pulse", d0, df, 0.0, qf, ascending, false, negated)
{
  if (d0.toSec() == 0.0 || df.toSec() >= INFINITY)
  {
    throw utilities::Exception(
        "Use the step function instead of the pulse one.");
  }
}

template <typename T>
PulseFunction<T>::PulseFunction(const StepFunction<T>& step_function, double df)
    : Function<T>::Function("Pulse", step_function, false)
{
  Function<T>::df_ = df;
}

template <typename T>
PulseFunction<T>::PulseFunction(const StepFunction<T>& step_function,
                                ros::Duration df)
  : Function<T>::Function("Pulse", step_function, false)
{
  Function<T>::df_ = df.toSec();
}

template <typename T>
PulseFunction<T>::PulseFunction(double d0, double df, double q0, double qf,
                                bool ascending, bool negated)
    : Function<T>::Function("Pulse", d0, df, q0, qf, ascending, negated, false)
{
  if (d0 == 0.0 || df >= INFINITY)
  {
    throw utilities::Exception(
        "Use the step function instead of the pulse one.");
  }
}

template <typename T>
PulseFunction<T>::PulseFunction(ros::Duration d0, ros::Duration df, double q0,
                                double qf, bool ascending, bool negated)
    : Function<T>::Function("Pulse", d0, df, q0, qf, ascending, negated, false)
{
  if (d0.toSec() == 0.0 || df.toSec() >= INFINITY)
  {
    throw utilities::Exception(
        "Use the step function instead of the pulse one.");
  }
}

template <typename T>
PulseFunction<T>::PulseFunction(const PulseFunction<T>& function)
    : Function<T>::Function(function)
{
}

template <typename T> PulseFunction<T>::~PulseFunction() {}

template <typename T> double PulseFunction<T>::calculate(double d) const
{
  return d <= Function<T>::d0_ || d > Function<T>::df_ ? Function<T>::q0_
                                                       : Function<T>::qf_;
}
}
}

#endif // _UTILITIES_PULSE_FUNCTION_H_
