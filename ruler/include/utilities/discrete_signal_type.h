/**
 *  This header file defines the DiscreteSignalType class, which is based on the
 *NonUnarySignalType abstract class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_DISCRETE_SIGNAL_TYPE_H_
#define _UTILITIES_DISCRETE_SIGNAL_TYPE_H_

#include "utilities/non_unary_signal_type.h"

namespace utilities
{
class DiscreteSignalType : public NonUnarySignalType<long>
{
public:
  DiscreteSignalType(long value);
  DiscreteSignalType(const DiscreteSignalType& signal_type);
  virtual ~DiscreteSignalType();
  virtual DiscreteSignalType& operator++();
  virtual DiscreteSignalType& operator++(int);
  virtual DiscreteSignalType operator--();
  virtual DiscreteSignalType operator--(int);
};
}

#endif // _UTILITIES_DISCRETE_SIGNAL_TYPE_H_