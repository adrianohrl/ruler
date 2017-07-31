/**
 *  This source file implements the DiscreteSignalType class, which is based on
 *the NonUnarySignalType abstract class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include <cmath>
#include "utilities/discrete_signal_type.h"

namespace utilities
{

DiscreteSignalType::DiscreteSignalType(long value)
    : SignalType<long>::SignalType(value)
{
}

DiscreteSignalType::DiscreteSignalType(float value)
    : SignalType<long>::SignalType(round(value))
{
}

DiscreteSignalType::DiscreteSignalType(double value)
    : SignalType<long>::SignalType(round(value))
{
}

DiscreteSignalType::DiscreteSignalType(const DiscreteSignalType& signal_type)
    : SignalType<long>::SignalType(signal_type)
{
}

DiscreteSignalType::~DiscreteSignalType() {}

DiscreteSignalType& DiscreteSignalType::operator-() { value_ = -value_; }

DiscreteSignalType& DiscreteSignalType::operator++()
{
  value_++;
  return *this;
}

DiscreteSignalType& DiscreteSignalType::operator++(int)
{
  ++value_;
  return *this;
}

DiscreteSignalType DiscreteSignalType::operator--()
{
  value_--;
  return *this;
}

DiscreteSignalType DiscreteSignalType::operator--(int)
{
  --value_;
  return *this;
}
}
