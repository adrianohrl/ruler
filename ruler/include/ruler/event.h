/**
 *  This header file defines and implements the Event class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _RESOURCE_CONTROL_EVENT_H_
#define _RESOURCE_CONTROL_EVENT_H_

#include <ros/time.h>
#include "ruler/event_type.h"

namespace ruler
{
template <typename T> class Event
{
public:
  Event(EventType type);
  virtual ~Virtual();
  ros::Time getTime() const;
  EventType getType() const;

private:
  ros::Time time_;
  EventType type_;
};

template <typename T>
Event<T>::Event(EventType type)
    : type_(type), time_(ros::Time::now())
{
}

template <typename T> Event<T>::~Virtual() {}

template <typename T> ros::Time Event<T>::getTime() const {}

template <typename T> EventType Event<T>::getType() const {}
}

#endif // _RESOURCE_CONTROL_EVENT_H_
