#include "utilities/toggle_event.h"

namespace utilities
{
ToggleEvent::ToggleEvent(Subject *subject, bool value,
                         ros::Time timestamp)
    : Event::Event(subject, timestamp), value_(value)
{
}

ToggleEvent::ToggleEvent(const ToggleEvent& event)
    : Event::Event(event), value_(event.value_)
{
}

ToggleEvent::~ToggleEvent() {}

bool ToggleEvent::getValue() const { return value_; }
}
