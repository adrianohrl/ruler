/**
 *  This header file defines the UnarySignalType class, which is based on the
 *SignalType abstract class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_UNARY_SIGNAL_TYPE_H_
#define _UTILITIES_UNARY_SIGNAL_TYPE_H_

#include "utilities/signal_type.h"

namespace utilities
{
class UnarySignalType : public SignalType<bool>
{
public:
  UnarySignalType(bool value);
  UnarySignalType(const UnarySignalType& signal_type);
  virtual ~UnarySignalType();
  virtual std::string str() const;
  virtual bool operator!() const;
};
}

#endif // _UTILITIES_UNARY_SIGNAL_TYPE_H_