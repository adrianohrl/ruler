/**
 *  This header file implements the TaskEvent class, which is based on the Event
 *class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "ruler/task_event.h"
#include "ruler/task.h"

namespace ruler
{

TaskEvent::TaskEvent(const TaskPtr& task, const EventType& type,
                     const ros::Time& timestamp)
    : Event::Event(task, timestamp), type_(type)
{
}

TaskEvent::TaskEvent(const TaskEvent& event)
    : Event::Event(event), type_(event.type_)
{
}

TaskEvent::~TaskEvent() {}

TaskPtr TaskEvent::getTask() const
{
  return boost::dynamic_pointer_cast<Task>(subject_);
}

EventType TaskEvent::getType() const { return type_; }
}
