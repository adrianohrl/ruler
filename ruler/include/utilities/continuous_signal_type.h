/**
 *  This header file defines the ContinuousSignalType interface.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_CONTINUOUS_SIGNAL_TYPE_H_
#define _UTILITIES_CONTINUOUS_SIGNAL_TYPE_H_

#include "utilities/signal_type.h"

namespace utilities
{
class ContinuousSignalType : public SignalType<double>
{
public:
  virtual double getValue() const = 0;
};
}

#endif // _UTILITIES_CONTINUOUS_SIGNAL_TYPE_H_
