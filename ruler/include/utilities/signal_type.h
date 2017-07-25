/**
 *  This header file defines the SignalType abstract class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_SIGNAL_TYPE_H_
#define _UTILITIES_SIGNAL_TYPE_H_

#include <sstream>

namespace utilities
{
template <typename T> class SignalType
{
public:
  virtual ~SignalType();
  T getValue() const;
  void setValue(const T& value);
  virtual std::string str() const;
  const char* c_str() const;
  virtual bool operator==(const T& value) const;
  virtual bool operator==(const SignalType<T>& signal_type) const;
  virtual bool operator!=(const T& value) const;
  virtual bool operator!=(const SignalType<T>& signal_type) const;
  virtual SignalType<T>& operator=(const T& value);
  virtual SignalType<T>& operator=(const SignalType<T>& signal_type);

protected:
  SignalType(const T& value);
  SignalType(const SignalType<T>& signal_type);
  T value_;
};

template <typename T> SignalType<T>::SignalType(const T& value) : value_(value)
{
}

template <typename T>
SignalType<T>::SignalType(const SignalType<T>& signal_type)
    : value_(signal_type.value_)
{
}

template <typename T>
SignalType<T>::~SignalType() {}

template <typename T> T SignalType<T>::getValue() const { return value_; }

template <typename T> void SignalType<T>::setValue(const T& value)
{
  value_ = value;
}

template <typename T>
std::string SignalType<T>::str() const
{
  std::stringstream ss;
  ss << value_;
  return ss.str();
}

template <typename T>
const char* SignalType<T>::c_str() const
{
  return str().c_str();
}

template <typename T> bool SignalType<T>::operator==(const T& value) const
{
  return value_ == value;
}

template <typename T>
bool SignalType<T>::operator==(const SignalType<T>& signal_type) const
{
  return value_ == signal_type.value_;
}

template <typename T> bool SignalType<T>::operator!=(const T& value) const
{
  return value_ != value;
}

template <typename T>
bool SignalType<T>::operator!=(const SignalType<T>& signal_type) const
{
  return value_ != signal_type.value_;
}

template <typename T> SignalType<T>& SignalType<T>::operator=(const T& value)
{
  value_ = value;
  return *this;
}

template <typename T>
SignalType<T>& SignalType<T>::operator=(const SignalType<T>& signal_type)
{
  value_ = signal_type.value_;
  return *this;
}
}

#endif // _UTILITIES_SIGNAL_TYPE_H_
