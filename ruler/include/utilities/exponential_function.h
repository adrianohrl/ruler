/**
 *  This header file defines and implements the ExponentialFunction class, which
 *is based on the Function abstract class.
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
template <typename T> class ExponentialFunction : public Function<T>
{
public:
  ExponentialFunction(double d0, double df, T q0, T qf, bool ascending = false,
                      double k = 5, double base = M_E);
  ExponentialFunction(ros::Duration d0, ros::Duration df, T q0, T qf,
                      bool ascending = false, double k = 5, double base = M_E);
  ExponentialFunction(const ExponentialFunction<T>& function);
  virtual ~ExponentialFunction();

private:
  double base_;
  double k_;
  virtual T calculate(double d) const;
};

template <typename T>
ExponentialFunction<T>::ExponentialFunction(double d0, double df, T q0, T qf,
                                            bool ascending, double k,
                                            double base)
    : Function<T>::Function(d0, df, q0, qf, ascending), base_(base), k_(fabs(k))
{
}

template <typename T>
ExponentialFunction<T>::ExponentialFunction(ros::Duration d0, ros::Duration df,
                                            T q0, T qf, bool ascending,
                                            double k, double base)
    : Function<T>::Function(d0, df, q0, qf, ascending), base_(base), k_(fabs(k))
{
}

template <typename T>
ExponentialFunction<T>::ExponentialFunction(
    const ExponentialFunction<T>& function)
    : Function<T>::Function(function)
{
}

template <typename T> ExponentialFunction<T>::~ExponentialFunction() {}

template <typename T> T ExponentialFunction<T>::calculate(double d) const
{
  double rate(-k_ / (Function<T>::df_ - Function<T>::d0_));
  return (T)(Function<T>::qf_ -
             (Function<T>::qf_ - Function<T>::q0_) *
                 pow(base_, rate * (d - Function<T>::d0_)));
}
}

#endif // _UTILITIES_EXPONENTIAL_FUNCTION_H_
