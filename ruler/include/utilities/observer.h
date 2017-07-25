/**
 *  This header file defines and implements the Observer class of the Observer
 *Design Pattern.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_OBSERVER_H_
#define _UTILITIES_OBSERVER_H_

namespace utilities
{
template <typename T> class Observer
{
public:
  virtual void update(T* notification) = 0;
  virtual void update(const T& notification) = 0;
};
}

#endif // _UTILITIES_OBSERVER_H_
