/**
 *  This header file implements the TaskEvent class, which is based on the Event
 *class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "ruler/task.h"
#include "ruler/task_event.h"

namespace ruler
{

TaskEvent::TaskEvent(Task* task, EventType type, ros::Time timestamp)
    : Event::Event(task, timestamp), type_(type)
{
}

TaskEvent::TaskEvent(const TaskEvent& event)
    : Event::Event(event), type_(event.type_)
{
}

TaskEvent::~TaskEvent() {}

Task* TaskEvent::getTask() const
{
  return (Task*) utilities::Event::getSubject();
}

EventType TaskEvent::getType() const { return type_; }
}
