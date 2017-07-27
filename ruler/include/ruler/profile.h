/**
 *  This header file defines and implements the Resource Profile class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _RULER_RESOURCE_PROFILE_H_
#define _RULER_RESOURCE_PROFILE_H_

#include <list>
#include <ros/common.h>
#include <ros/time.h>
#include "ruler/task_function.h"

namespace ruler
{
template <typename T> class Profile
{
public:
  Profile(T capacity, T initial_level);
  Profile(const Profile<T>& profile);
  virtual ~Profile();
  T getCapacity() const;
  T getInitialLevel() const;
  T getLevel(ros::Time t = ros::Time::now()) const;
  double getLevel(double d, double initial) const;
  void update(const TaskEvent& notification);
  void addTaskFunction(TaskFunction* task_function);
  void removeTaskFunction(Task* task);

private:
  T capacity_;
  T initial_level_;
  std::list<TaskFunction*> task_functions_;
};

template <typename T>
Profile<T>::Profile(T capacity, T initial_level)
    : capacity_(capacity), initial_level_(initial_level)
{
  if (initial_level_ > capacity_)
  {
    throw utilities::Exception("Initial level must be less than or equal to the resource capacity.");
  }
}

template <typename T>
Profile<T>::Profile(const Profile<T>& profile)
    : capacity_(profile.capacity_), initial_level_(profile.initial_level_)
{
}

template <typename T> Profile<T>::~Profile()
{
  std::list<TaskFunction*>::iterator it(task_functions_.begin());
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

template <typename T> T Profile<T>::getCapacity() const { return capacity_; }

template <typename T> T Profile<T>::getInitialLevel() const
{
  return initial_level_;
}

template <typename T> T Profile<T>::getLevel(ros::Time t) const
{
  T level(initial_level_);
  //ROS_WARN_STREAM("[PROFILE] initial level: " << level);
  std::list<TaskFunction*>::const_iterator it(task_functions_.begin());
  while (it != task_functions_.end())
  {
    TaskFunction* task_function = *it;
    level += task_function->getLevel(t);
    //ROS_WARN_STREAM("[PROFILE] level: " << level);
    it++;
  }
  //ROS_WARN_STREAM("[PROFILE] final level: " << level);
  return level;
}

template <typename T> double Profile<T>::getLevel(double d, double initial_level) const
{
  double level(initial_level);
  //ROS_WARN_STREAM("[PROFILE] initial level: " << level);
  std::list<TaskFunction*>::const_iterator it(task_functions_.begin());
  while (it != task_functions_.end())
  {
    TaskFunction* task_function = *it;
    level += task_function->getLevel(d);
    //ROS_WARN_STREAM("[PROFILE] level: " << level);
    it++;
  }
  //ROS_WARN_STREAM("[PROFILE] final level: " << level);
  return level;
}

template <typename T> void Profile<T>::update(const TaskEvent& notification)
{
  std::list<TaskFunction*>::iterator it(task_functions_.begin());
  while (it != task_functions_.end())
  {
    TaskFunction* task_function = *it;
    if (task_function->getTask() == notification.getTask())
    {
      task_function->update(notification);
    }
    it++;
  }
}

template <typename T>
void Profile<T>::addTaskFunction(TaskFunction* task_function)
{
  task_functions_.push_back(task_function);
}

template <typename T>
void Profile<T>::removeTaskFunction(Task* task)
{
  std::list<TaskFunction*>::iterator it(task_functions_.begin());
  while (it != task_functions_.end())
  {
    TaskFunction* task_function = *it;
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
