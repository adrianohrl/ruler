/**
 *  This header file defines and implements the Task class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _RESOURCE_CONTROL_TASK_FUNCTION_H_
#define _RESOURCE_CONTROL_TASK_FUNCTION_H_

#include <list>
#include <ros/time.h>
#include "ruler/task.h"
#include "ruler/event.h"
#include "utilities/function.h"
#include "utilities/observer.h"

namespace ruler
{
template <typename T> class TaskFunction : public utilities::Observer<Event<T> >
{
public:
  TaskFunction(Task* task, utilities::Function* quantity_function);
  virtual ~TaskFunction();
  double estimate(ros::Time t) const;

private:
  Task* task_;
  std::list<Event<T>*> events_;
  utilities::Function* quantity_;
};

template <typename T>
TaskFunction<T>::TaskFunction(Task* task,
                              utilities::Function* quantity_function)
    : task_(task), quantity_(quantity_function)
{
}

template <typename T> TaskFunction<T>::~TaskFunction() { task_ = NULL; }

template <typename T> double TaskFunction<T>::estimate(ros::Time t) const
{
  /*double level(0.0);
  std::list<Event<T>*>::const_iterator it(events_.begin());
  while (it != events_.end())
  {
    Event<T>* event = *it;
    level += quantity_->getValue(task_->getDuration(t));
    it++;
  }*/
  return quantity_->getValue(task_->getDuration(t));
}
}

#endif // _RESOURCE_CONTROL_TASK_FUNCTION_H_
