/**
 *  This header file implements the TaskFunction class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#include "ruler/task.h"
#include "ruler/task_function.h"

namespace ruler
{
TaskFunction::TaskFunction(Task* task, utilities::Function* quantity_function)
    : task_(task), quantity_function_(quantity_function)
{
}

TaskFunction::~TaskFunction()
{
  task_ = NULL;
  if (quantity_function_)
  {
    delete quantity_function_;
    quantity_function_ = NULL;
  }
}

void TaskFunction::update(const TaskEvent& notification)
{
  events_.push_back(notification);
}

double TaskFunction::getLevel(ros::Time t) const
{
  /*double level(0.0);
  std::list<Event*>::const_iterator it(events_.begin());
  while (it != events_.end())
  {
    Event* event = *it;
    level += quantity_->getValue(task_->getDuration(t));
    it++;
  }*/
  return quantity_function_->getValue(task_->getDuration(t));
}

double TaskFunction::getLevel(double d) const
{
  return quantity_function_->getValue(d);
}

Task* TaskFunction::getTask() const { return task_; }
}
