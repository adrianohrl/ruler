#ifndef _ALLIANCE_BEHAVIOUR_SET_INTERFACE_H_
#define _ALLIANCE_BEHAVIOUR_SET_INTERFACE_H_

#include "alliance/task.h"
#include <utilities/subject.h>
#include <utilities/functions/unary_sample_holder.h>

namespace alliance
{
template <typename R, typename BS>
class BehaviourSetInterface : public boost::enable_shared_from_this<BS>
{

  typedef boost::shared_ptr<R> RPtr;

public:
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
  const RPtr robot_;
  const TaskPtr task_;
  ros::Time activation_timestamp_;
  const utilities::functions::UnarySampleHolderPtr active_;
};

template <typename R, typename BS>
BehaviourSetInterface<R, BS>::BehaviourSetInterface(
    const RPtr& robot, const TaskPtr& task, const ros::Duration& buffer_horizon,
    const ros::Duration& timeout_duration)
    : robot_(robot), task_(task),
      active_(new utilities::functions::UnarySampleHolder(
          robot->getId() + "/" + task->getId() + "/active", timeout_duration,
          buffer_horizon))
{
}

template <typename R, typename BS>
BehaviourSetInterface<R, BS>::~BehaviourSetInterface()
{
}

template <typename R, typename BS>
void BehaviourSetInterface<R, BS>::preProcess()
{
}

template <typename R, typename BS> void BehaviourSetInterface<R, BS>::process()
{
}

template <typename R, typename BS>
TaskPtr BehaviourSetInterface<R, BS>::getTask() const
{
  return task_;
}

template <typename R, typename BS>
bool BehaviourSetInterface<R, BS>::isActive(const ros::Time& timestamp) const
{
  return active_->getValue(timestamp);
}

template <typename R, typename BS>
ros::Time BehaviourSetInterface<R, BS>::getActivationTimestamp() const
{
  return activation_timestamp_;
}

template <typename R, typename BS>
void BehaviourSetInterface<R, BS>::setActive(bool active,
                                             const ros::Time& timestamp)
{
  if (active != active_->getValue(timestamp))
  {
    ROS_WARN_STREAM("Updating " << *active_ << " to " << active << ".");
    active_->update(active, timestamp);
    activation_timestamp_ = active ? timestamp : ros::Time();
  }
}

template <typename R, typename BS>
void BehaviourSetInterface<R, BS>::setTimeoutDuration(
    const ros::Duration& timeout_duration)
{
  active_->setTimeoutDuration(timeout_duration);
}

template <typename R, typename BS>
void BehaviourSetInterface<R, BS>::setBufferHorizon(
    const ros::Duration& buffer_horizon)
{
  active_->setBufferHorizon(buffer_horizon);
}
}
#endif // _ALLIANCE_BEHAVIOUR_SET_INTERFACE_H_
