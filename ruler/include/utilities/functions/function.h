/**
 * This header file defines and implements the Function class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_FUNCTION_H_
#define _UTILITIES_FUNCTION_H_

#include <sstream>
#include <ros/duration.h>
#include "utilities/exception.h"

namespace utilities
{
namespace functions
{
template <typename T> class Function
{
public:
  virtual ~Function();
  T getValue(double d) const;
  std::string getName() const;
  bool isAscending() const;
  bool isNegated() const;
  void setAscending(bool ascending);
  void setNegated(bool negated);
  bool saturatesEnd() const;
  std::string str() const;
  const char* c_str() const;

protected:
  Function(std::string name, double d0, double df, double q0, double qf,
           bool ascending, bool negated, bool saturate_end = true);
  Function(std::string name, ros::Duration d0, ros::Duration df, double q0,
           double qf, bool ascending, bool negated, bool saturate_end = true);
  Function(std::string name, const Function<T>& function,
           bool saturate_end = true);
  Function(const Function<T>& function);
  double d0_;
  double df_;
  double q0_;
  double qf_;

private:
  const std::string name_;
  bool ascending_;
  bool negated_;
  const bool saturate_end_;
  virtual double calculate(double d) const = 0;
};

template <typename T>
Function<T>::Function(std::string name, double d0, double df, double q0,
                      double qf, bool ascending, bool negated,
                      bool saturate_end)
    : name_(name), d0_(d0), df_(df), q0_(q0), qf_(qf), ascending_(ascending),
      negated_(negated), saturate_end_(saturate_end)
{
}

template <typename T>
Function<T>::Function(std::string name, ros::Duration d0, ros::Duration df,
                      double q0, double qf, bool ascending, bool negated,
                      bool saturate_end)
    : name_(name), d0_(d0.toSec()), df_(df.toSec()), q0_(q0), qf_(qf),
      ascending_(ascending), negated_(negated), saturate_end_(saturate_end)
{
}

template <typename T>
Function<T>::Function(std::string name, const Function<T>& function,
                      bool saturate_end)
    : name_(name), d0_(function.d0_), df_(function.df_), q0_(function.q0_),
      qf_(function.qf_), ascending_(function.ascending_),
      negated_(function.negated_), saturate_end_(saturate_end)
{
}

template <typename T>
Function<T>::Function(const Function<T>& function)
    : name_(function.name_), d0_(function.d0_), df_(function.df_),
      q0_(function.q0_), qf_(function.qf_), ascending_(function.ascending_),
      negated_(function.negated_), saturate_end_(function.saturate_end_)
{
}

template <typename T> Function<T>::~Function() {}

template <typename T> T Function<T>::getValue(double d) const
{
  double q;
  if (d <= d0_)
  {
    q = q0_;
  }
  else
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
  if (saturate_end_ && d > df_)
  {
    q = qf_;
  }
  return ascending_ ? q : qf_ - q + q0_;
}

template <typename T> std::string Function<T>::getName() const { return name_; }

template <typename T> bool Function<T>::isAscending() const
{
  return ascending_;
}

template <typename T> bool Function<T>::isNegated() const { return negated_; }

template <typename T> void Function<T>::setAscending(bool ascending)
{
  ascending_ = ascending;
}

template <typename T> void Function<T>::setNegated(bool negated)
{
  negated_ = negated;
}

template <typename T> bool Function<T>::saturatesEnd() const
{
  return saturate_end_;
}

template <typename T> std::string Function<T>::str() const
{
  std::stringstream ss;
  ss << name_ << " Function: ";
  ss << "d0 = " << d0_ << ", df = " << df_;
  ss << ", q0 = " << q0_ << ", qf = " << qf_;
  ss << (ascending_ ? " (asc)" : " (dec)");
  ss << (negated_ ? " (neg)" : " (pos)");
  return ss.str();
}

template <typename T> const char *Function<T>::c_str() const
{
  return str().c_str();
}
}
}

#endif // _UTILITIES_FUNCTION_H_