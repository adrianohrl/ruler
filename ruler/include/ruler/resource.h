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

namespace ruler
{
template <typename T> class Resource : public utilities::Observer<Event>
{
public:
  virtual ~Resource();
  virtual void update(Event* notification);
  virtual void update(const Event& notification);
  std::string getType() const;
  std::string getName() const;
  T getCapacity() const;
  T getInitialLevel() const;
  T getLevel() const;
  ros::Duration getLatence() const;
  Profile<T>* getProfile() const;
  void setCapacity(T capacity);
  std::string str() const;
  bool operator==(const Resource<T>& resource) const;
  bool operator!=(const Resource<T>& resource) const;
  void operator+(const T& level);
  void operator-(const T& level);

protected:
  Resource(std::string type, std::string name, T capacity, T initial_level,
           ros::Duration latence = ros::Duration(0.0));
  Resource(const Resource<T>& resource);

private:
  std::string type_;
  std::string name_;
  T capacity_;
  T level_;
  T initial_level_;
  ros::Duration latence_;
  Profile<T>* profile_;
};

template <typename T>
Resource<T>::Resource(std::string type, std::string name, T capacity,
                      T initial_level, ros::Duration latence)
    : type_(type), name_(name), capacity_(capacity), level_(initial_level),
      initial_level_(initial_level), latence_(latence),
      profile_(new Profile<T>())
{
}

template <typename T>
Resource<T>::Resource(const Resource<T> &resource)
  : type_(resource.type_), name_(resource.name_), capacity_(resource.capacity_),
    level_(resource.level_), initial_level_(resource.initial_level_),
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

template <typename T> void Resource<T>::update(Event* notification)
{
  profile_->update(notification);
}

template <typename T> void Resource<T>::update(const Event& notification)
{
  profile_->update(notification);
}

template <typename T> std::string Resource<T>::getType() const { return type_; }

template <typename T> std::string Resource<T>::getName() const { return name_; }

template <typename T> T Resource<T>::getCapacity() const { return capacity_; }

template <typename T> T Resource<T>::getInitialLevel() const
{
  return initial_level_;
}

template <typename T> T Resource<T>::getLevel() const { return level_; }

template <typename T> ros::Duration Resource<T>::getLatence() const
{
  return latence_;
}

template <typename T> Profile<T>* Resource<T>::getProfile() const
{
  return profile_;
}

template <typename T> void Resource<T>::setCapacity(T capacity)
{
  capacity_ = capacity;
}

template <typename T> std::string Resource<T>::str() const { return name_; }

template <typename T>
bool Resource<T>::operator==(const Resource<T>& resource) const
{
  return type_ == resource.type_ && name_ == resource.name_;
}

template <typename T>
bool Resource<T>::operator!=(const Resource<T>& resource) const
{
  return !Resource<T>::operator==(resource);
}

template <typename T> void Resource<T>::operator+(const T& level)
{
  if (level_ + level <= capacity_)
  {
    level_ += level;
  }
}

template <typename T> void Resource<T>::operator-(const T& level)
{
  if (level_ >= level)
  {
    level_ -= level;
  }
}
}

#endif // _RULER_RESOURCE_H_
