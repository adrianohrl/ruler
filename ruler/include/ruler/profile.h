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

namespace ruler
{
template <typename T> class Profile
{
public:
  Profile(const T& capacity, const T& initial_level);
  Profile(const Profile<T>& profile);
  virtual ~Profile();
  bool isContinuous() const;
  bool isDiscrete() const;
  bool isUnary() const;
  T getCapacity() const;
  T getInitialLevel() const;
  T getLevel(ros::Time t = ros::Time::now()) const;
  void update(const TaskEventConstPtr &event);
  void addTaskFunction(TaskFunction<T>* task_function);
  void removeTaskFunction(Task* task);

private:
  T capacity_;
  T initial_level_;
  std::list<TaskFunction<T>*> task_functions_;
};

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

template <typename T> Profile<T>::~Profile()
{
  typename std::list<TaskFunction<T>*>::iterator it(task_functions_.begin());
  while (it != task_functions_.end())
  {
    if (*it)
    {
      delete *it;
      *it = NULL;
    }
    it++;
  }
}

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

template <typename T> T Profile<T>::getLevel(ros::Time t) const
{
  T level(initial_level_);
  typename std::list<TaskFunction<T>*>::const_iterator it(
      task_functions_.begin());
  while (it != task_functions_.end())
  {
    TaskFunction<T>* task_function = *it;
    if (task_function->isNegated())
    {
      level -= task_function->getLevel(t);
    }
    else
    {
      level += task_function->getLevel(t);
    }
    it++;
  }
  return level;
}

template <typename T> void Profile<T>::update(const TaskEventConstPtr& event)
{
  typename std::list<TaskFunction<T>*>::iterator it(task_functions_.begin());
  while (it != task_functions_.end())
  {
    TaskFunction<T>* task_function = *it;
    if (*task_function->getTask() == *event->getTask())
    {
      task_function->update(event);
    }
    it++;
  }
}

template <typename T>
void Profile<T>::addTaskFunction(TaskFunction<T>* task_function)
{
  task_functions_.push_back(task_function);
}

template <typename T> void Profile<T>::removeTaskFunction(Task* task)
{
  typename std::list<TaskFunction<T>*>::iterator it(task_functions_.begin());
  while (it != task_functions_.end())
  {
    TaskFunction<T>* task_function = *it;
    if (task_function->getTask() == task)
    {
      delete *it;
      *it = NULL;
      task_functions_.erase(it);
    }
    it++;
  }
}
}

#endif // _RULER_RESOURCE_PROFILE_H_
