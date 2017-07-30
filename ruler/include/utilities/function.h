/**
 * This header file defines and implements the Function class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_FUNCTION_H_
#define _UTILITIES_FUNCTION_H_

#include <string>
#include <ros/duration.h>
#include "utilities/exception.h"

namespace utilities
{
template <typename T> class Function
{
public:
  virtual ~Function();
  T getValue(double d) const;
  void setAscending(bool ascending);

protected:
  Function(double d0, double df, double q0, double qf, bool ascending = false);
  Function(ros::Duration d0, ros::Duration df, double q0, double qf,
           bool ascending = false);
  Function(const Function<T>& function);
  double d0_;
  double df_;
  double q0_;
  double qf_;

private:
  bool ascending_;
  virtual double calculate(double d) const = 0;
};

template <typename T>
Function<T>::Function(double d0, double df, double q0, double qf, bool ascending)
    : d0_(d0), df_(df), q0_(q0), qf_(qf), ascending_(ascending)
{
}

template <typename T>
Function<T>::Function(ros::Duration d0, ros::Duration df, double q0, double qf,
                      bool ascending)
    : d0_(d0.toSec()), df_(df.toSec()), q0_(q0), qf_(qf), ascending_(ascending)
{
}

template <typename T>
Function<T>::Function(const Function<T>& function)
    : d0_(function.d0_), df_(function.df_), q0_(function.q0_),
      qf_(function.qf_), ascending_(function.ascending_)
{
}

template <typename T> Function<T>::~Function() {}

template <typename T> T Function<T>::getValue(double d) const
{
  double q;
  if (d < d0_)
  {
    q = q0_;
  }
  else if (d <= df_)
  {
    q = calculate(d);
    if (q < q0_)
    {
      q = q0_;
    }
    else if (q > qf_)
    {
      q = qf_;
    }
  }
  else
  {
    q = qf_;
  }
  return ascending_ ? q : qf_ - q + q0_;
}

template <typename T> void Function<T>::setAscending(bool ascending)
{
  ascending_ = ascending;
}
}

#endif // _UTILITIES_FUNCTION_H_
