/**
 *  This header file defines and implements the Resource abstract class.
 *
 *  Author: Adriano Henrique Rossette Leite (adrianohrl@unifei.edu.br)
 *  Maintainer: Expertinos UNIFEI (expertinos.unifei@gmail.com)
 */

#ifndef _RULER_RESOURCE_H_
#define _RULER_RESOURCE_H_

#include <ros/duration.h>
#include "ruler/profile.h"
#include "ruler/resource_interface.h"
#include <typeinfo>

namespace ruler
{
template <typename T> class Resource : public ResourceInterface
{
public:
  virtual ~Resource();
  virtual void update(const utilities::EventConstPtr& event);
  virtual void update(const TaskEventConstPtr& event);
  virtual bool isConsumable() const;
  virtual bool isReusable() const;
  virtual bool isContinuous() const;
  virtual bool isDiscrete() const;
  virtual bool isUnary() const;
  virtual utilities::SignalTypeEnum getSignalType() const;
  std::string getName() const;
  T getLevel(ros::Time timestamp = ros::Time::now()) const;
  ros::Duration getLatence() const;
  virtual ruler_msgs::Resource toMsg() const;

protected:
  Profile<T>* profile_;
  Resource(std::string id, std::string name, T capacity, T initial_level,
           ros::Duration latence = ros::Duration(0.0));
  Resource(const ruler_msgs::Resource& msg);
  Resource(const Resource<T>& resource);

private:
  const std::string name_;
  ros::Duration latence_;
};

template <typename T>
Resource<T>::Resource(std::string id, std::string name, T capacity,
                      T initial_level, ros::Duration latence)
    : ResourceInterface::ResourceInterface(id), name_(name), latence_(latence),
      profile_(new Profile<T>(capacity, initial_level))
{
  if (name_.empty())
  {
    throw utilities::Exception("Resource name must not be empty.");
  }
  if (latence_.toSec() < 0.0)
  {
    throw utilities::Exception("Resource latence must not be negative.");
  }
}

template <typename T>
Resource<T>::Resource(const ruler_msgs::Resource& msg)
    : ResourceInterface::ResourceInterface(msg.header.frame_id),
      name_(msg.name), latence_(msg.latence),
      profile_(new Profile<T>(msg.capacity, msg.level))
{
  if (name_.empty())
  {
    throw utilities::Exception("Resource name must not be empty.");
  }
  if (latence_.toSec() < 0.0)
  {
    throw utilities::Exception("Resource latence must not be negative.");
  }
}

template <typename T>
Resource<T>::Resource(const Resource<T>& resource)
    : ResourceInterface::ResourceInterface(resource), name_(resource.name_),
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

template <typename T> void Resource<T>::update(const utilities::EventConstPtr& event)
{
  if (typeid(*event) == typeid(TaskEvent))
  {
    Resource<T>::update(boost::dynamic_pointer_cast<TaskEvent const>(event));
  }
}

template <typename T> void Resource<T>::update(const TaskEventConstPtr& event)
{
  profile_->update(event);
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

template <typename T>
utilities::SignalTypeEnum Resource<T>::getSignalType() const
{
  if (isContinuous())
  {
    return utilities::signal_types::CONTINUOUS;
  }
  else if (isDiscrete())
  {
    return utilities::signal_types::DISCRETE;
  }
  return utilities::signal_types::UNARY;
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

template <typename T> ruler_msgs::Resource Resource<T>::toMsg() const
{
  ruler_msgs::Resource msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = getId();
  msg.signal_type = utilities::SignalTypes::toCode(getSignalType());
  msg.consumable = isConsumable();
  msg.name = name_;
  msg.latence = latence_;
  msg.capacity = profile_->getCapacity();
  msg.level = getLevel(msg.header.stamp);
  return msg;
}
}

#endif // _RULER_RESOURCE_H_
