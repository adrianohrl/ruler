/**
 *  This header file defines the Step class, which is based on the Function
 *abstract class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_STEP_H_
#define _UTILITIES_STEP_H_

#include "utilities/function.h"

namespace utilities
{
class Step : Function
{
public:
  Step(double qf);
  virtual ~Step();

private:
  virtual double calculate(double d) const;
};
}

#endif // _UTILITIES_STEP_H_
