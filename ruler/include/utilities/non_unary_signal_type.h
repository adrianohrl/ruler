/**
  *  This header file defines the NonUnarySignalType abstract class, which is
  *based on the SignalType abstract class.
  *
  *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
  *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
  */

#ifndef _UTILITIES_NON_UNARY_SIGNAL_TYPE_H_
#define _UTILITIES_NON_UNARY_SIGNAL_TYPE_H_

#include "utilities/signal_type.h"

namespace utilities
{
template <typename T> class NonUnarySignalType : public SignalType<T>
{
public:
  virtual ~NonUnarySignalType();
  virtual NonUnarySignalType<T>& operator-();

protected:
  NonUnarySignalType(const T& value);
  NonUnarySignalType(const NonUnarySignalType<T>& signal_type);
};

template <typename T> NonUnarySignalType<T>::NonUnarySignalType(const T& value)
  : SignalType<T>::SignalType(value)
{
}

template <typename T>
NonUnarySignalType<T>::NonUnarySignalType(const NonUnarySignalType<T>& signal_type)
    : SignalType<T>::SignalType(signal_type)
{
}

template <typename T> NonUnarySignalType<T>::~NonUnarySignalType() {}

template <typename T> NonUnarySignalType<T>& NonUnarySignalType<T>::operator-()
{
  SignalType<T>::value_ = -SignalType<T>::value_;
  return *this;
}
}

#endif // _UTILITIES_NON_UNARY_SIGNAL_TYPE_H_
