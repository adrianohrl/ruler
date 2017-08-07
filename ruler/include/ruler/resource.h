/**
 *  This header file defines and implements the Resource abstract class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _RULER_RESOURCE_H_
#define _RULER_RESOURCE_H_

#include <string>
#include <ros/duration.h>
#include "ruler/profile.h"
#include "utilities/exception.h"
#include "utilities/observer.h"

namespace ruler
{
template <typename T> class Resource : public utilities::Observer<TaskEvent>
{
public:
  virtual ~Resource();
  virtual void update(const TaskEvent& notification);
  virtual bool isConsumable() const;
  virtual bool isReusable() const;
  bool isContinuous() const;
  bool isDiscrete() const;
  bool isUnary() const;
  std::string getName() const;
  T getLevel(ros::Time timestamp = ros::Time::now()) const;
  ros::Duration getLatence() const;

protected:
  Profile<T>* profile_;
  Resource(std::string id, std::string name, T capacity, T initial_level,
           ros::Duration latence = ros::Duration(0.0));
  Resource(const Resource<T>& resource);

private:
  std::string name_;
  ros::Duration latence_;
};

template <typename T>
Resource<T>::Resource(std::string id, std::string name, T capacity,
                      T initial_level, ros::Duration latence)
    : utilities::Observer<TaskEvent>::Observer(id), name_(name),
      latence_(latence), profile_(new Profile<T>(capacity, initial_level))
{
}

template <typename T>
Resource<T>::Resource(const Resource<T>& resource)
    : utilities::Observer<TaskEvent>::Observer(resource), name_(resource.name_),
      latence_(resource.latence_), profile_(resource.profile_)
{
}

template <typename T> Resource<T>::~Resource()
{
  if (profile_)
  {
    delete profile_;
    profile_ = NULL;
  }
}

template <typename T> void Resource<T>::update(const TaskEvent& notification)
{
  profile_->update(notification);
}

template <typename T> bool Resource<T>::isConsumable() const { return false; }

template <typename T> bool Resource<T>::isReusable() const { return false; }

template <typename T> bool Resource<T>::isContinuous() const
{
  return profile_->isContinuous();
}

template <typename T> bool Resource<T>::isDiscrete() const
{
  return profile_->isDiscrete();
}

template <typename T> bool Resource<T>::isUnary() const
{
  return profile_->isUnary();
}

template <typename T> std::string Resource<T>::getName() const { return name_; }

template <typename T> T Resource<T>::getLevel(ros::Time timestamp) const
{
  return profile_->getLevel(timestamp);
}

template <typename T> ros::Duration Resource<T>::getLatence() const
{
  return latence_;
}
}

#endif // _RULER_RESOURCE_H_
