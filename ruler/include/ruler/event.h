/**
 *  This header file defines the Event class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _RULER_EVENT_H_
#define _RULER_EVENT_H_

#include <ros/time.h>
#include "ruler/event_types.h"

namespace ruler
{
class Event
{
public:
  Event(EventType type, ros::Time timestamp = ros::Time::now());
  Event(const Event& event);
  virtual ~Event();
  ros::Time getTimestamp() const;
  EventType getType() const;

private:
  ros::Time timestamp_;
  EventType type_;
};
}

#endif // _RULER_EVENT_H_
