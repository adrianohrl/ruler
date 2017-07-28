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

TaskEvent::TaskEvent(Task* task, EventType type)
    : Event::Event(type), task_(task)
{
}

TaskEvent::TaskEvent(const TaskEvent& event)
    : Event::Event(event), task_(event.task_)
{
}

TaskEvent::~TaskEvent() { task_ = NULL; }

Task* TaskEvent::getTask() const { return task_; }
}
