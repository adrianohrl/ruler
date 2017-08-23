/**
 *  This header file defines the TaskEvent class, which is based on the Event
 *class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _RULER_TASK_EVENT_H_
#define _RULER_TASK_EVENT_H_

#include "ruler/event_types.h"
#include "utilities/event.h"

namespace ruler
{
class Task;

class TaskEvent : public utilities::Event
{
public:
  TaskEvent(Task* task, EventType type,
            ros::Time timestamp = ros::Time::now());
  TaskEvent(const TaskEvent& event);
  virtual ~TaskEvent();
  Task* getTask() const;
  EventType getType() const;

private:
  EventType type_;
};
}

#endif // _RULER_TASK_EVENT_H_
