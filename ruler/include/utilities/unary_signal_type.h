/**
 *  This header file defines the UnarySignalType interface.
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
  virtual bool getValue() const;
};
}

#endif // _UTILITIES_UNARY_SIGNAL_TYPE_H_
