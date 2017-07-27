/**
 * This header file defines the Function class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_FUNCTION_H_
#define _UTILITIES_FUNCTION_H_

#include <string>
#include "utilities/exception.h"

namespace utilities
{
class Function
{
public:
  virtual ~Function();
  double getValue(double d) const;
  void setAscending(bool ascending);

protected:
  Function(double d0, double df, double q0, double qf, bool ascending = false);
  Function(const Function& function);
  double d0_;
  double df_;
  double q0_;
  double qf_;

private:
  bool ascending_;
  virtual double calculate(double d) const = 0;
};
}

#endif // _UTILITIES_FUNCTION_H_
