/**
 *  This header file defines the SignalType interface.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_SIGNAL_TYPE_H_
#define _UTILITIES_SIGNAL_TYPE_H_

namespace utilities
{
template <typename T> class SignalType
{
public:
  virtual T getValue() const = 0;
};
}

#endif // _UTILITIES_SIGNAL_TYPE_H_
