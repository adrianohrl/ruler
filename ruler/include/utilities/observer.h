/**
 *  This header file defines and implements the Observer abstract class of the
 *Observer Design Pattern.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_OBSERVER_H_
#define _UTILITIES_OBSERVER_H_

#include "utilities/has_id.h"

namespace utilities
{
template <typename T> class Observer : public HasId<std::string, T>
{
public:
  virtual ~Observer();
  virtual void update(const T& notification) = 0;

protected:
  Observer(std::string id);
  Observer(const Observer<T>& observer);
};

template <typename T>
Observer<T>::Observer(std::string id)
    : HasId<std::string, T>::HasId(id)
{
}

template <typename T>
Observer<T>::Observer(const Observer<T>& observer)
    : HasId<std::string, T>::HasId(observer)
{
}

template <typename T> Observer<T>::~Observer() {}
}

#endif // _UTILITIES_OBSERVER_H_