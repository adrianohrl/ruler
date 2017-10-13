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
#include "utilities/continuous_signal_type.h"
#include "utilities/discrete_signal_type.h"
#include "utilities/functions/pulse_function.h"
#include "utilities/unary_signal_type.h"

namespace ruler
{
template <typename T> class Resource;

template <typename T> class TaskFunction
{
protected:
  typedef typename boost::shared_ptr<Resource<T>> ResourcePtr;
  typedef typename boost::shared_ptr<Resource<T> const> ResourceConstPtr;
  typedef typename utilities::functions::Function<T>::Ptr FunctionPtr;
  typedef typename utilities::functions::Function<T>::ConstPtr FunctionConstPtr;

public:
  typedef boost::shared_ptr<TaskFunction<T>> Ptr;
  typedef boost::shared_ptr<TaskFunction<T> const> ConstPtr;
  TaskFunction(const ResourcePtr& resource, const TaskPtr& task,
               const FunctionPtr& quantity_function);
  TaskFunction(const TaskFunction<T>& task_function);
  virtual ~TaskFunction();
  void update(const TaskEventConstPtr& event);
  bool isNegated() const;
  T getLevel(const ros::Time& timestmap) const;
  ResourcePtr getResource() const;
  TaskPtr getTask() const;

private:
  typedef typename std::list<FunctionPtr>::iterator iterator;
  typedef typename std::list<FunctionPtr>::const_iterator const_iterator;
  ResourcePtr resource_;
  TaskPtr task_;
  FunctionPtr quantity_function_;
  std::list<FunctionPtr> interrupted_quantity_functions_;
};

typedef TaskFunction<utilities::ContinuousSignalType> ContinuousTaskFunction;
typedef TaskFunction<utilities::ContinuousSignalType>::Ptr
    ContinuousTaskFunctionPtr;
typedef TaskFunction<utilities::ContinuousSignalType>::ConstPtr
    ContinuousTaskFunctionConstPtr;
typedef TaskFunction<utilities::DiscreteSignalType> DiscreteTaskFunction;
typedef TaskFunction<utilities::DiscreteSignalType>::Ptr
    DiscreteTaskFunctionPtr;
typedef TaskFunction<utilities::DiscreteSignalType>::ConstPtr
    DiscreteTaskFunctionConstPtr;
typedef TaskFunction<utilities::UnarySignalType> UnaryTaskFunction;
typedef TaskFunction<utilities::UnarySignalType>::Ptr UnaryTaskFunctionPtr;
typedef TaskFunction<utilities::UnarySignalType>::ConstPtr
    UnaryTaskFunctionConstPtr;
}

#include "ruler/resource.h"
#include "ruler/task.h"
#include "ruler/preemptive_task.h"

namespace ruler
{
template <typename T>
TaskFunction<T>::TaskFunction(const ResourcePtr& resource, const TaskPtr& task,
                              const FunctionPtr& quantity_function)
    : resource_(resource), task_(task), quantity_function_(quantity_function)
{
}

template <typename T>
TaskFunction<T>::TaskFunction(const TaskFunction<T>& task_function)
    : resource_(task_function.resource_), task_(task_function.task_),
      quantity_function_(task_function.quantity_function_)
{
}

template <typename T> TaskFunction<T>::~TaskFunction() {}

template <typename T>
void TaskFunction<T>::update(const TaskEventConstPtr& event)
{
  typedef utilities::functions::StepFunction<T> StepFunction;
  typedef typename utilities::functions::StepFunction<T>::Ptr StepFunctionPtr;
  typedef typename utilities::functions::StepFunction<T>::ConstPtr
      StepFunctionConstPtr;
  typedef utilities::functions::PulseFunction<T> PulseFunction;
  typedef typename utilities::functions::PulseFunction<T>::Ptr PulseFunctionPtr;
  typedef typename utilities::functions::PulseFunction<T>::ConstPtr
      PulseFunctionConstPtr;
  if (resource_->isReusable())
  {
    if (task_->isPreemptive())
    {
      PreemptiveTaskPtr task(
          boost::dynamic_pointer_cast<PreemptiveTask>(task_));
      if (event->getType() == types::INTERRUPTED)
      {
        ros::Time timestamp(task->getLastInterruptionTimestamp());
        ros::Duration d0(task->getDuration(timestamp));
        double qf(getLevel(timestamp));
        bool ascending(quantity_function_->isAscending());
        bool negated(!quantity_function_->isNegated());
        FunctionPtr interrupted_quantity_function(
            new utilities::functions::StepFunction<T>(d0, qf, ascending,
                                                      negated));
        interrupted_quantity_functions_.push_back(
            interrupted_quantity_function);
      }
      else if (event->getType() == types::RESUMED)
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
        StepFunctionPtr last_interrupted_quantity_function(
            boost::dynamic_pointer_cast<StepFunction>(
                interrupted_quantity_functions_.back()));
        ros::Duration df(task->getLastResumeTimestamp() -
                         task->getStartTimestamp());
        FunctionPtr interrupted_quantity_function(
            new PulseFunction(*last_interrupted_quantity_function, df));
        interrupted_quantity_functions_.erase(
            --interrupted_quantity_functions_.end());
        interrupted_quantity_functions_.push_back(
            interrupted_quantity_function);
      }
    }
    else if (event->getType() == types::INTERRUPTED ||
             event->getType() == types::RESUMED)
    {
      throw utilities::Exception(
          "Non-preemptive task cannot be interrupted ot resumed.");
    }
    else if (event->getType() == types::FINISHED)
    {
      ROS_WARN("[TaskFunction] TERMINOU!!!");
      ros::Time timestamp(task_->getEndTimestamp());
      ros::Duration d0(timestamp - task_->getStartTimestamp());
      double qf(getLevel(timestamp));
      bool ascending(quantity_function_->isAscending());
      bool negated(!quantity_function_->isNegated());
      FunctionPtr interrupted_quantity_function(
          new StepFunction(d0, qf, ascending, negated));
      interrupted_quantity_functions_.push_back(interrupted_quantity_function);
    }
  }
}

template <typename T> bool TaskFunction<T>::isNegated() const
{
  return quantity_function_->isNegated();
}

template <typename T>
T TaskFunction<T>::getLevel(const ros::Time& timestmap) const
{
  T level(quantity_function_->getValue(task_->getDuration(timestmap).toSec()));
  double duration((timestmap - task_->getStartTimestamp()).toSec());
  const_iterator it(interrupted_quantity_functions_.begin());
  while (it != interrupted_quantity_functions_.end())
  {
    FunctionPtr interrupted_quantity_function(*it);
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

template <typename T>
boost::shared_ptr<Resource<T>> TaskFunction<T>::getResource() const
{
  return resource_;
}

template <typename T> TaskPtr TaskFunction<T>::getTask() const { return task_; }
}

#endif // _RULER_TASK_FUNCTION_H_
