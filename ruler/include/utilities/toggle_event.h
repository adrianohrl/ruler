#ifndef _UTILITIES_TOGGLE_EVENT_H_
#define _UTILITIES_TOGGLE_EVENT_H_

#include "utilities/event.h"

namespace utilities
{
class ToggleEvent : public Event
{
public:
  ToggleEvent(Subject* subject, bool value,
              ros::Time timestamp = ros::Time::now());
  ToggleEvent(const ToggleEvent& event);
  virtual ~ToggleEvent();
  bool getValue() const;

private:
  bool value_;
};
}

#endif // _UTILITIES_TOGGLE_EVENT_H_
