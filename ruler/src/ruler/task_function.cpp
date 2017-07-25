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
    : task_(task), quantity_(quantity_function)
{
}

TaskFunction::~TaskFunction() {
  task_ = NULL;
  if (quantity_)
  {
    delete quantity_;
    quantity_ = NULL;
  }
}

double TaskFunction::estimate(ros::Time t) const
{
  /*double level(0.0);
  std::list<Event*>::const_iterator it(events_.begin());
  while (it != events_.end())
  {
    Event* event = *it;
    level += quantity_->getValue(task_->getDuration(t));
    it++;
  }*/
  return quantity_->getValue(task_->getDuration(t));
}

void TaskFunction::update(Event* notification)
{
  events_.push_back(*notification);
}

void TaskFunction::update(const Event& notification)
{
  events_.push_back(notification);
}

std::string TaskFunction::str() const
{
  return "";
}
}
