/**
 *  This header file defines and implements the Subject class of the Observer
 *Design Pattern.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _UTILITIES_SUBJECT_H_
#define _UTILITIES_SUBJECT_H_

#include <list>
#include "utilities/observer.h"

namespace utilities
{

template <typename T> class Subject
{
public:
  virtual ~Subject();
  void registerObserver(Observer<T>* observer);
  void unregisterObserver(Observer<T>* observer);

protected:
  void notify(T* notification);
  void notify(const T& notification);

private:
  std::list<Observer<T>*> observers_;
};

template <typename T> Subject<T>::~Subject() {}

template <typename T>
void Subject<T>::registerObserver(Observer<T>* observer)
{
  observers_.push_back(observers_);
}

template <typename T>
void Subject<T>::unregisterObserver(Observer<T>* observer)
{
  observers_.remove(observer);
}

template <typename T> void Subject<T>::notify(T* notification)
{
  std::list<Observer<T>*>::iterator it(observers_.begin());
  while (it != observers_.end())
  {
    Observer<T>* observer = *it;
    observer->update(notification);
    it++;
  }
}

template <typename T> void Subject<T>::notify(const T& notification)
{
  std::list<Observer<T>*>::iterator it(observers_.begin());
  while (it != observers_.end())
  {
    Observer<T>* observer = *it;
    observer->update(notification);
    it++;
  }
}
}

#endif // _UTILITIES_SUBJECT_H_
