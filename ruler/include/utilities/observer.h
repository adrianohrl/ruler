/**
 *  This header file defines and implements the Observer abstract class of the
 *Observer Design Pattern.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_OBSERVER_H_
#define _UTILITIES_OBSERVER_H_

#include <string>

namespace utilities
{
template <typename T> class Observer
{
public:
  virtual void update(T* notification) = 0;
  virtual void update(const T& notification) = 0;
  virtual std::string str() const = 0;
  virtual const char* c_str() const;
};

template <typename T> const char *Observer<T>::c_str() const
{
  return str().c_str();
}

}

#endif // _UTILITIES_OBSERVER_H_
