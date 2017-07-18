/**
 *  This header file defines the DiscreteSignal interface.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_DISCRETE_SIGNAL_H_
#define _UTILITIES_DISCRETE_SIGNAL_H_

#include "utilities/signal_type.h"

namespace utilities
{
class DiscreteSignal : public SignalType<long>
{
public:
  virtual long getValue() const = 0;
};
}

#endif // _UTILITIES_DISCRETE_SIGNAL_H_
