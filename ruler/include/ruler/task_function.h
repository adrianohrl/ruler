/**
 *  This header file defines and implements the TaskFunction class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _RULER_TASK_FUNCTION_H_
#define _RULER_TASK_FUNCTION_H_

#include <list>
#include <ros/time.h>
#include "ruler/task_event.h"
#include "utilities/functions/pulse_function.h"

namespace ruler
{
template <typename T> class Resource;

template <typename T> class TaskFunction
{
public:
  TaskFunction(Resource<T>* resource, Task* task,
               utilities::functions::Function<T>* quantity_function);
  TaskFunction(const TaskFunction<T>& task_function);
  virtual ~TaskFunction();
  void update(const TaskEvent& event);
  bool isNegated() const;
  T getLevel(ros::Time t) const;
  Resource<T>* getResource() const;
  Task* getTask() const;

private:
  Resource<T>* resource_;
  Task* task_;
  std::list<TaskEvent> events_;
  utilities::functions::Function<T>* quantity_function_;
  std::list<utilities::functions::Function<T>*> interrupted_quantity_functions_;
};
}

#include "ruler/resource.h"
#include "ruler/task.h"

namespace ruler
{
template <typename T>
TaskFunction<T>::TaskFunction(
    Resource<T>* resource, Task* task,
    utilities::functions::Function<T>* quantity_function)
    : resource_(resource), task_(task), quantity_function_(quantity_function)
{
}

template <typename T>
TaskFunction<T>::TaskFunction(const TaskFunction<T>& task_function)
    : resource_(task_function.resource_), task_(task_function.task_),
      quantity_function_(task_function.quantity_function_)
{
}

template <typename T> TaskFunction<T>::~TaskFunction()
{
  resource_ = NULL;
  task_ = NULL;
  if (quantity_function_)
  {
    delete quantity_function_;
    quantity_function_ = NULL;
  }
  typename std::list<utilities::functions::Function<T>*>::iterator it(
      interrupted_quantity_functions_.begin());
  while (it != interrupted_quantity_functions_.end())
  {
    if (*it)
    {
      delete *it;
      *it = NULL;
    }
    it++;
  }
}

template <typename T>
void TaskFunction<T>::update(const TaskEvent& event)
{
  if (resource_->isReusable())
  {
    if (event.getType() == types::INTERRUPTED)
    {
      ros::Time timestamp(task_->getLastInterruptionTimestamp());
      double d0(task_->getDuration(timestamp));
      double qf(getLevel(timestamp));
      bool ascending(quantity_function_->isAscending());
      bool negated(!quantity_function_->isNegated());
      utilities::functions::Function<T>* interrupted_quantity_function =
          new utilities::functions::StepFunction<T>(d0, qf, ascending, negated);
      interrupted_quantity_functions_.push_back(interrupted_quantity_function);
    }
    else if (event.getType() == types::RESUMED)
    {
      if (interrupted_quantity_functions_.empty())
      {
        throw utilities::Exception("Unable to resume task function. The "
                                   "interrupted_quantity_functions list is "
                                   "empty.");
      }
      if (interrupted_quantity_functions_.back()->getName() != "Step")
      {
        throw utilities::Exception(
            "Unable to resume task function. The las element of the "
            "interrupted_quantity_functions list is not a step function.");
      }
      utilities::functions::StepFunction<T>*
          last_interrupted_quantity_function =
              (utilities::functions::StepFunction<T>*)
                  interrupted_quantity_functions_.back();
      ros::Duration df(task_->getLastResumeTimestamp() -
                       task_->getStartTimestamp());
      utilities::functions::Function<T>* interrupted_quantity_function =
          new utilities::functions::PulseFunction<T>(
              *last_interrupted_quantity_function, df);
      delete last_interrupted_quantity_function;
      last_interrupted_quantity_function = NULL;
      interrupted_quantity_functions_.erase(
          --interrupted_quantity_functions_.end());
      interrupted_quantity_functions_.push_back(interrupted_quantity_function);
    }
    else if (event.getType() == types::FINISHED)
    {
      ros::Time timestamp(task_->getEndTimestamp());
      ros::Duration d0(task_->getEndTimestamp() - task_->getStartTimestamp());
      double qf(getLevel(timestamp));
      bool ascending(quantity_function_->isAscending());
      bool negated(!quantity_function_->isNegated());
      utilities::functions::Function<T>* interrupted_quantity_function =
          new utilities::functions::StepFunction<T>(d0, qf, ascending, negated);
      interrupted_quantity_functions_.push_back(interrupted_quantity_function);
    }
  }
  events_.push_back(event);
}

template <typename T> bool TaskFunction<T>::isNegated() const
{
  return quantity_function_->isNegated();
}

template <typename T> T TaskFunction<T>::getLevel(ros::Time t) const
{
  T level(quantity_function_->getValue(task_->getDuration(t)));
  double duration((t - task_->getStartTimestamp()).toSec());
  typename std::list<utilities::functions::Function<T>*>::const_iterator it(
      interrupted_quantity_functions_.begin());
  while (it != interrupted_quantity_functions_.end())
  {
    utilities::functions::Function<T>* interrupted_quantity_function = *it;
    if (interrupted_quantity_function->isNegated())
    {
      level += interrupted_quantity_function->getValue(duration);
    }
    else
    {
      level -= interrupted_quantity_function->getValue(duration);
    }
    it++;
  }
  return level;
}

template <typename T> Resource<T>* TaskFunction<T>::getResource() const
{
  return resource_;
}

template <typename T> Task* TaskFunction<T>::getTask() const { return task_; }
}

#endif // _RULER_TASK_FUNCTION_H_
