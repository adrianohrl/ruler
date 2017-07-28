/**
 *  This header file defines and implements the StepFunction class, which is
 *based on the Function abstract class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_STEP_FUNCTION_H_
#define _UTILITIES_STEP_FUNCTION_H_

#include "utilities/function.h"

namespace utilities
{
template <typename T> class StepFunction : public Function<T>
{
public:
  StepFunction(T qf);
  StepFunction(double d0, double df, T q0, T qf, bool ascending = false);
  StepFunction(ros::Duration d0, ros::Duration df, T q0, T qf, bool ascending = false);
  StepFunction(const StepFunction<T>& function);
  virtual ~StepFunction();

private:
  virtual T calculate(double d) const;
};

template <typename T>
StepFunction<T>::StepFunction(T qf)
  : Function<T>::Function(0.0, INFINITY, T(), qf, true)
{
}

template <typename T>
StepFunction<T>::StepFunction(double d0, double df, T q0, T qf, bool ascending)
    : Function<T>::Function(d0, df, q0, qf, ascending)
{
}

template <typename T>
StepFunction<T>::StepFunction(ros::Duration d0, ros::Duration df, T q0, T qf, bool ascending)
    : Function<T>::Function(d0, df, q0, qf, ascending)
{
}

template <typename T>
StepFunction<T>::StepFunction(const StepFunction<T>& function)
    : Function<T>::Function(function)
{
}

template <typename T> StepFunction<T>::~StepFunction() {}

template <typename T> T StepFunction<T>::calculate(double d) const
{
  return Function<T>::qf_;
}
}

#endif // _UTILITIES_STEP_FUNCTION_H_
