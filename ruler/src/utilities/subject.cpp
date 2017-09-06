/**
 *  This source file implements and implements the Subject class of the Observer
 *Design Pattern.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "utilities/subject.h"

namespace utilities
{
Subject::Subject(const std::string& id) : HasId<std::string>::HasId(id) {}

Subject::Subject(const Subject& subject) : HasId<std::string>::HasId(subject) {}

Subject::~Subject() {}

void Subject::registerObserver(const ObserverPtr& observer)
{
  observers_.push_back(observer);
  ROS_DEBUG_STREAM("Registered observer (" << *observer << ") to subject ("
                                           << *this << ").");
}

void Subject::unregisterObserver(const ObserverPtr& observer)
{
  observers_.remove(observer);
  ROS_DEBUG_STREAM("Unregistered observer (" << *observer << ") to subject ("
                                             << *this << ").");
}

void Subject::clearObservers() { observers_.clear(); }

void Subject::notify(const EventConstPtr& event)
{
  std::list<ObserverPtr>::iterator it(observers_.begin());
  while (it != observers_.end())
  {
    ObserverPtr observer(*it);
    observer->update(event);
    ROS_DEBUG_STREAM("Subject (" << *this << ") notified observer ("
                                 << *observer << ").");
    it++;
  }
}

bool Subject::empty() const { return observers_.empty(); }
}
