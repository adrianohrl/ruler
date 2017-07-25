/**
 *  This header file implements the Event class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "ruler/event.h"

namespace ruler
{
Event::Event(EventType type) : type_(type), timestamp_(ros::Time::now()) {}

Event::Event(const Event& event)
    : type_(event.type_), timestamp_(event.timestamp_)
{
}

Event::~Event() {}

ros::Time Event::getTimestamp() const {}

EventType Event::getType() const {}
}
