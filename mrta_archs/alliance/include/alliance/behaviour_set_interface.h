#ifndef _ALLIANCE_BEHAVIOUR_SET_INTERFACE_H_
#define _ALLIANCE_BEHAVIOUR_SET_INTERFACE_H_

#include "alliance/task.h"
#include <utilities/subject.h>
#include <utilities/functions/unary_sample_holder.h>

namespace alliance
{
template <typename R>
class BehaviourSetInterface
{
public:
  typedef boost::shared_ptr<R> RPtr;
  BehaviourSetInterface(
      const RPtr& robot, const TaskPtr& task,
      const ros::Duration& buffer_horizon,
      const ros::Duration& timeout_duration = ros::Duration());
  virtual ~BehaviourSetInterface();
  virtual void preProcess();
  virtual void process();
  TaskPtr getTask() const;
  bool isActive(const ros::Time& timestamp = ros::Time::now()) const;
  ros::Time getActivationTimestamp() const;
  virtual void setActive(bool active = true,
                         const ros::Time& timestamp = ros::Time::now());
  void setTimeoutDuration(const ros::Duration& timeout_duration);
  void setBufferHorizon(const ros::Duration& buffer_horizon);

protected:
  typedef utilities::functions::UnarySampleHolder SampleHolder;
  typedef utilities::functions::UnarySampleHolderPtr SampleHolderPtr;
  const RPtr robot_;
  const TaskPtr task_;
  ros::Time activation_timestamp_;
  SampleHolderPtr active_;
};

template <typename R>
BehaviourSetInterface<R>::BehaviourSetInterface(
    const RPtr& robot, const TaskPtr& task, const ros::Duration& buffer_horizon,
    const ros::Duration& timeout_duration)
    : robot_(robot), task_(task),
      active_(new SampleHolder(
          robot->getId() + "/" + task->getId() + "/active", timeout_duration,
          buffer_horizon))
{
}

template <typename R>
BehaviourSetInterface<R>::~BehaviourSetInterface()
{
}

template <typename R>
void BehaviourSetInterface<R>::preProcess()
{
}

template <typename R> void BehaviourSetInterface<R>::process()
{
}

template <typename R>
TaskPtr BehaviourSetInterface<R>::getTask() const
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
    ROS_WARN_STREAM("[BSI] Updating " << *active_ << " to " << active << ".");
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
