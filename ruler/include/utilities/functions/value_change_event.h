#ifndef _UTILITIES_FUNCTIONS_VALUE_CHANGE_EVENT_H_
#define _UTILITIES_FUNCTIONS_VALUE_CHANGE_EVENT_H_

#include "utilities/event.h"

namespace utilities
{
namespace functions
{
template <typename T> class ValueChangeEvent : public Event
{
public:
  ValueChangeEvent(Subject* subject, T value,
                   ros::Time timestamp = ros::Time::now());
  ValueChangeEvent(const ValueChangeEvent& event);
  virtual ~ValueChangeEvent();
  const T getValue() const;

private:
  const T value_;
};

template <typename T>
ValueChangeEvent<T>::ValueChangeEvent(Subject* subject, T value,
                                      ros::Time timestamp)
    : Event::Event(subject, timestamp), value_(value)
{
}

template <typename T>
ValueChangeEvent<T>::ValueChangeEvent(const ValueChangeEvent& event)
    : Event::Event(event), value_(event.value_)
{
}

template <typename T> ValueChangeEvent<T>::~ValueChangeEvent() {}

template <typename T> const T ValueChangeEvent<T>::getValue() const
{
  return value_;
}
}
}

#endif // _UTILITIES_FUNCTIONS_VALUE_CHANGE_EVENT_H_
