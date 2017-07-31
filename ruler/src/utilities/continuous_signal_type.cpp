/**
 *  This source file implements the ContinuousSignalType class, which is based
 *on the NonUnarySignalType abstract class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "utilities/continuous_signal_type.h"

namespace utilities
{

ContinuousSignalType::ContinuousSignalType(double value)
    : SignalType<double>::SignalType(value)
{
}

ContinuousSignalType::ContinuousSignalType(
    const ContinuousSignalType& signal_type)
    : SignalType<double>::SignalType(signal_type)
{
}

ContinuousSignalType::~ContinuousSignalType() {}

ContinuousSignalType& ContinuousSignalType::operator-()
{
  value_ = -value_;
  return *this;
}
}
