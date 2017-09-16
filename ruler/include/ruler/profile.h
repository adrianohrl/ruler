/**
 *  This header file defines and implements the Resource Profile class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _RULER_RESOURCE_PROFILE_H_
#define _RULER_RESOURCE_PROFILE_H_

#include <list>
#include "ruler/task_function.h"
#include "utilities/continuous_signal_type.h"
#include "utilities/discrete_signal_type.h"
#include "utilities/unary_signal_type.h"

namespace ruler
{
template <typename T> class Profile
{
protected:
  typedef typename TaskFunction<T>::Ptr TaskFunctionPtr;
  typedef typename TaskFunction<T>::ConstPtr TaskFunctionConstPtr;

public:
  typedef boost::shared_ptr<Profile<T>> Ptr;
  typedef boost::shared_ptr<Profile<T> const> ConstPtr;
  Profile(const T& capacity, const T& initial_level);
  Profile(const Profile<T>& profile);
  virtual ~Profile();
  bool isContinuous() const;
  bool isDiscrete() const;
  bool isUnary() const;
  T getCapacity() const;
  T getInitialLevel() const;
  T getLevel(const ros::Time& timestamp = ros::Time::now()) const;
  void update(const TaskEventConstPtr& event);
  void addTaskFunction(const TaskFunctionPtr& task_function);
  void removeTaskFunction(const TaskPtr& task);

private:
  typedef typename std::list<TaskFunctionPtr>::iterator iterator;
  typedef typename std::list<TaskFunctionPtr>::const_iterator const_iterator;
  T capacity_;
  T initial_level_;
  std::list<TaskFunctionPtr> task_functions_;
};

typedef Profile<utilities::ContinuousSignalType> ContinuousProfile;
typedef Profile<utilities::ContinuousSignalType>::Ptr ContinuousProfilePtr;
typedef Profile<utilities::ContinuousSignalType>::ConstPtr ContinuousProfileConstPtr;
typedef Profile<utilities::DiscreteSignalType> DiscreteProfile;
typedef Profile<utilities::DiscreteSignalType>::Ptr DiscreteProfilePtr;
typedef Profile<utilities::DiscreteSignalType>::ConstPtr DiscreteProfileConstPtr;
typedef Profile<utilities::UnarySignalType> UnaryProfile;
typedef Profile<utilities::UnarySignalType>::Ptr UnaryProfilePtr;
typedef Profile<utilities::UnarySignalType>::ConstPtr UnaryProfileConstPtr;

template <typename T>
Profile<T>::Profile(const T& capacity, const T& initial_level)
    : capacity_(capacity), initial_level_(initial_level)
{
  if (initial_level_ > capacity_)
  {
    throw utilities::Exception(
        "Initial level must be less than or equal to the resource capacity.");
  }
}

template <typename T>
Profile<T>::Profile(const Profile<T>& profile)
    : capacity_(profile.capacity_), initial_level_(profile.initial_level_)
{
}

template <typename T> Profile<T>::~Profile() {}

template <typename T> bool Profile<T>::isContinuous() const
{
  return capacity_.isContinuous();
}

template <typename T> bool Profile<T>::isDiscrete() const
{
  return capacity_.isDiscrete();
}

template <typename T> bool Profile<T>::isUnary() const
{
  return capacity_.isUnary();
}

template <typename T> T Profile<T>::getCapacity() const { return capacity_; }

template <typename T> T Profile<T>::getInitialLevel() const
{
  return initial_level_;
}

template <typename T> T Profile<T>::getLevel(const ros::Time& timestamp) const
{
  T level(initial_level_);
  for (const_iterator it(task_functions_.begin()); it != task_functions_.end();
       it++)
  {
    TaskFunctionPtr task_function(*it);
    if (task_function->isNegated())
    {
      level -= task_function->getLevel(timestamp);
    }
    else
    {
      level += task_function->getLevel(timestamp);
    }
  }
  return level;
}

template <typename T> void Profile<T>::update(const TaskEventConstPtr& event)
{
  for (iterator it(task_functions_.begin()); it != task_functions_.end(); it++)
  {
    TaskFunctionPtr task_function(*it);
    if (*task_function->getTask() == *event->getTask())
    {
      task_function->update(event);
    }
  }
}

template <typename T>
void Profile<T>::addTaskFunction(const TaskFunctionPtr& task_function)
{
  task_functions_.push_back(task_function);
}

template <typename T> void Profile<T>::removeTaskFunction(const TaskPtr& task)
{
  for (iterator it(task_functions_.begin()); it != task_functions_.end(); it++)
  {
    TaskFunctionPtr task_function(*it);
    if (*task_function->getTask() == task)
    {
      task_functions_.erase(it);
    }
  }
}
}

#endif // _RULER_RESOURCE_PROFILE_H_
