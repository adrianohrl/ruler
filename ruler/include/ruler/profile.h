/**
 *  This header file defines and implements the Resource Profile class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _RULER_RESOURCE_PROFILE_H_
#define _RULER_RESOURCE_PROFILE_H_

#include <list>
#include <ros/time.h>
#include "ruler/task_function.h"
#include "utilities/observer.h"

namespace ruler
{
template<typename T>
class Profile : public utilities::Observer<Event>
{
public:
  Profile();
  Profile(const Profile<T>& profile);
  virtual ~Profile();
  T estimate(ros::Time t = ros::Time::now()) const;
  virtual void update(Event* notification);
  virtual void update(const Event& notification);

private:
  std::list<TaskFunction*> task_functions_;
};

template<typename T>
Profile<T>::Profile()
{

}

template<typename T>
Profile<T>::Profile(const Profile<T> &profile)
{

}

template<typename T>
Profile<T>::~Profile()
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

template<typename T>
T Profile<T>::estimate(ros::Time t) const
{
  double level(0.0);
  std::list<TaskFunction*>::const_iterator it(task_functions_.begin());
  while (it != task_functions_.end())
  {
    TaskFunction* task_function = *it;
    level += task_function->estimate(t);
    it++;
  }
  return level;
}

template<typename T>
void Profile<T>::update(Event *notification)
{

}

template<typename T>
void Profile<T>::update(const Event &notification)
{

}

}

#endif // _RULER_RESOURCE_PROFILE_H_
