#ifndef _ALLIANCE_BEHAVIOUR_SET_INTERFACE_H_
#define _ALLIANCE_BEHAVIOUR_SET_INTERFACE_H_

#include "alliance/task.h"
#include <utilities/subject.h>
#include <utilities/functions/unary_sample_holder.h>

namespace alliance
{
template <typename R> class BehaviourSetInterface
{
public:
  BehaviourSetInterface(R* robot, Task* task, ros::Duration buffer_horizon,
                        ros::Duration timeout_duration = ros::Duration());
  virtual ~BehaviourSetInterface();
  virtual void preProcess();
  virtual void process();
  Task* getTask() const;
  bool isActive(const ros::Time& timestamp = ros::Time::now()) const;
  ros::Time getActivationTimestamp() const;
  virtual void setActive(bool active = true,
                         const ros::Time& timestamp = ros::Time::now());
  void setTimeoutDuration(const ros::Duration& timeout_duration);
  void setBufferHorizon(const ros::Duration& buffer_horizon);

protected:
  R* robot_;
  Task* task_;
  ros::Time activation_timestamp_;
  utilities::functions::UnarySampleHolder* active_;
};

template <typename R>
BehaviourSetInterface<R>::BehaviourSetInterface(R* robot, Task* task,
                                                ros::Duration buffer_horizon,
                                                ros::Duration timeout_duration)
    : robot_(robot), task_(task)
{
  active_ = new utilities::functions::UnarySampleHolder(
      robot->getId() + "/" + task->getId() + "/active", timeout_duration,
      buffer_horizon);
}

template <typename R> BehaviourSetInterface<R>::~BehaviourSetInterface()
{
  robot_ = NULL;
  task_ = NULL;
  if (active_)
  {
    delete active_;
    active_ = NULL;
  }
}

template <typename R> void BehaviourSetInterface<R>::preProcess() {}

template <typename R> void BehaviourSetInterface<R>::process() {}

template <typename R> Task* BehaviourSetInterface<R>::getTask() const
{
  return task_;
}

template <typename R>
bool BehaviourSetInterface<R>::isActive(const ros::Time& timestamp) const
{
  return active_->getValue(timestamp);
}

template <typename R>
ros::Time BehaviourSetInterface<R>::getActivationTimestamp() const
{
  return activation_timestamp_;
}

template <typename R>
void BehaviourSetInterface<R>::setActive(bool active,
                                         const ros::Time& timestamp)
{
  if (active != active_->getValue(timestamp))
  {
    ROS_WARN_STREAM("Updating " << *active_ << " to " << active << ".");
    active_->update(active, timestamp);
    activation_timestamp_ = active ? timestamp : ros::Time();
  }
}

template <typename R>
void BehaviourSetInterface<R>::setTimeoutDuration(
    const ros::Duration& timeout_duration)
{
  active_->setTimeoutDuration(timeout_duration);
}

template <typename R>
void BehaviourSetInterface<R>::setBufferHorizon(
    const ros::Duration& buffer_horizon)
{
  active_->setBufferHorizon(buffer_horizon);
}
}
#endif // _ALLIANCE_BEHAVIOUR_SET_INTERFACE_H_
