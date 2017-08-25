/**
 *  This header file defines and implements the StepFunction class, which is
 *based on the Function abstract class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_STEP_FUNCTION_H_
#define _UTILITIES_STEP_FUNCTION_H_

#include "utilities/functions/function.h"

namespace utilities
{
namespace functions
{
template <typename T> class StepFunction : public Function<T>
{
public:
  StepFunction(double qf, bool ascending, bool negated);
  StepFunction(double d0, double qf, bool ascending, bool negated);
  StepFunction(ros::Duration d0, double qf, bool ascending, bool negated);
  StepFunction(const StepFunction<T>& function);
  virtual ~StepFunction();
  virtual StepFunction<T>* clone() const;

protected:
  StepFunction(double d0, double q0, double qf, bool ascending, bool negated);
  StepFunction(ros::Duration d0, double q0, double qf, bool ascending,
               bool negated);

private:
  virtual double calculate(double d) const;
};

template <typename T>
StepFunction<T>::StepFunction(double qf, bool ascending, bool negated)
    : Function<T>::Function("Step", 0.0, INFINITY, 0.0, qf, ascending, negated)
{
}

template <typename T>
StepFunction<T>::StepFunction(double d0, double qf, bool ascending,
                              bool negated)
    : Function<T>::Function("Step", d0, INFINITY, 0.0, qf, ascending, negated)
{
}

template <typename T>
StepFunction<T>::StepFunction(ros::Duration d0, double qf, bool ascending,
                              bool negated)
    : Function<T>::Function("Step", d0.toSec(), INFINITY, 0.0, qf, ascending,
                            negated)
{
}

template <typename T>
StepFunction<T>::StepFunction(double d0, double q0, double qf, bool ascending,
                              bool negated)
    : Function<T>::Function("Step", d0, INFINITY, q0, qf, ascending, negated)
{
}

template <typename T>
StepFunction<T>::StepFunction(ros::Duration d0, double q0, double qf,
                              bool ascending, bool negated)
    : Function<T>::Function("Step", d0.toSec(), INFINITY, q0, qf, ascending,
                            negated)
{
}

template <typename T>
StepFunction<T>::StepFunction(const StepFunction<T>& function)
    : Function<T>::Function(function)
{
}

template <typename T> StepFunction<T>::~StepFunction() {}

template <typename T> StepFunction<T> *StepFunction<T>::clone() const
{
  return new StepFunction<T>(*this);
}

template <typename T> double StepFunction<T>::calculate(double d) const
{
  return Function<T>::qf_;
}
}
}

#endif // _UTILITIES_STEP_FUNCTION_H_
