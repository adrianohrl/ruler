/**
 *  This header file defines the UnarySignal interface.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_UNARY_SIGNAL_H_
#define _UTILITIES_UNARY_SIGNAL_H_

#include "utilities/signal_type.h"

namespace utilities
{
class UnarySignal : public SignalType<bool>
{
public:
  virtual T getValue() const = 0;
};
}

#endif // _UTILITIES_UNARY_SIGNAL_H_
