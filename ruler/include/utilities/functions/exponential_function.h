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
#include "utilities/functions/function.h"

namespace utilities
{
namespace functions
{
template <typename T> class ExponentialFunction : public Function<T>
{
public:
  ExponentialFunction(double d0, double df, double q0, double qf, double k,
                      double base, bool ascending, bool negated);
  ExponentialFunction(ros::Duration d0, ros::Duration df, double q0, double qf,
                      double k, double base, bool ascending, bool negated);
  ExponentialFunction(const ExponentialFunction<T>& function);
  virtual ~ExponentialFunction();

private:
  double base_;
  double k_;
  virtual double calculate(double d) const;
};

template <typename T>
ExponentialFunction<T>::ExponentialFunction(double d0, double df, double q0,
                                            double qf, double k, double base,
                                            bool ascending, bool negated)
    : Function<T>::Function(d0, df, q0, qf, ascending, negated), base_(base),
      k_(fabs(k))
{
}

template <typename T>
ExponentialFunction<T>::ExponentialFunction(ros::Duration d0, ros::Duration df,
                                            double q0, double qf, double k,
                                            double base, bool ascending,
                                            bool negated)
    : Function<T>::Function(d0, df, q0, qf, ascending, negated), base_(base),
      k_(fabs(k))
{
}

template <typename T>
ExponentialFunction<T>::ExponentialFunction(
    const ExponentialFunction<T>& function)
    : Function<T>::Function(function)
{
}

template <typename T> ExponentialFunction<T>::~ExponentialFunction() {}

template <typename T> double ExponentialFunction<T>::calculate(double d) const
{
  double rate(-k_ / (Function<T>::df_ - Function<T>::d0_));
  return Function<T>::qf_ -
         (Function<T>::qf_ - Function<T>::q0_) *
             pow(base_, rate * (d - Function<T>::d0_));
}
}
}

#endif // _UTILITIES_EXPONENTIAL_FUNCTION_H_
