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
#include <ros/common.h>
#include "utilities/observer.h"

namespace utilities
{
template <typename T> class Subject : public HasId<std::string, T>
{
public:
  virtual ~Subject();

protected:
  Subject(std::string id);
  Subject(const Subject<T>& subject);
  void registerObserver(Observer<T>* observer);
  void unregisterObserver(Observer<T>* observer);
  void clearObservers();
  void notify(const T& notification);

private:
  std::list<Observer<T>*> observers_;
};

template <typename T>
Subject<T>::Subject(std::string id)
    : HasId<std::string, T>::HasId(id)
{
}

template <typename T>
Subject<T>::Subject(const Subject<T>& subject)
    : HasId<std::string, T>::HasId(subject)
{
}

template <typename T> Subject<T>::~Subject()
{
  typename std::list<Observer<T>*>::iterator it(observers_.begin());
  while (it != observers_.end())
  {
    *it = NULL;
    it++;
  }
  observers_.clear();
}

template <typename T> void Subject<T>::registerObserver(Observer<T>* observer)
{
  observers_.push_back(observer);
  ROS_DEBUG_STREAM("Registered observer (" << observer << ") to subject ("
                                           << *this << ").");
}

template <typename T> void Subject<T>::unregisterObserver(Observer<T>* observer)
{
  observers_.remove(observer);
  ROS_DEBUG_STREAM("Unregistered observer (" << observer << ") to subject ("
                                             << *this << ").");
}

template <typename T> void Subject<T>::clearObservers() { observers_.clear(); }

template <typename T> void Subject<T>::notify(const T& notification)
{
  typename std::list<Observer<T>*>::iterator it(observers_.begin());
  while (it != observers_.end())
  {
    Observer<T>* observer = *it;
    observer->update(notification);
    ROS_DEBUG_STREAM("Subject (" << *this << ") notified observer (" << observer
                                 << ").");
    it++;
  }
}
}

#endif // _UTILITIES_SUBJECT_H_
