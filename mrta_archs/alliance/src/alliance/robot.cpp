#include "alliance/robot.h"

namespace alliance
{
Robot::Robot(const std::string& id, const std::string& name)
    : RobotInterface::RobotInterface(name, id), broadcast_rate_(0.0),
      timeout_duration_(0.0)
{
}

Robot::~Robot() {}

void Robot::process()
{
  std::list<BehaviourSetInterface*>::iterator it(behaviour_sets_.begin());
  while (it != behaviour_sets_.end())
  {
    BehaviourSet* behaviour_set = (BehaviourSet*)*it;
    behaviour_set->process();
    if (behaviour_set->isActive())
    {
      if (active_behaviour_set_ && *active_behaviour_set_ != *behaviour_set)
      {
        ((BehaviourSet*)active_behaviour_set_)->setActive(false);
      }
      active_behaviour_set_ = behaviour_set;
      ((BehaviourSet*)active_behaviour_set_)->setActive();
      return;
    }
    it++;
  }
}

ros::Rate Robot::getBroadcastRate() const { return broadcast_rate_; }

ros::Duration Robot::getTimeoutDuration() const { return timeout_duration_; }

void Robot::setBroadcastRate(const ros::Rate& broadcast_rate)
{
  broadcast_rate_ = broadcast_rate;
}

void Robot::setTimeoutDuration(const ros::Duration& timeout_duration)
{
  timeout_duration_ = timeout_duration;
}

void Robot::addBehaviourSet(BehaviourSet* behaviour_set)
{
  if (!behaviour_set)
  {
    ROS_ERROR_STREAM("The given robot's behaviour set must not be null.");
    return;
  }
  if (contains(*behaviour_set))
  {
    ROS_WARN_STREAM(*this << " robot already have the " << *behaviour_set
                          << " behaviour set.");
    return;
  }
  std::list<BehaviourSetInterface*>::iterator it(behaviour_sets_.begin());
  while (it != behaviour_sets_.end())
  {
    BehaviourSet* robot_behaviour_set = (BehaviourSet*)*it;
    behaviour_set->registerActivitySuppression(robot_behaviour_set);
    robot_behaviour_set->registerActivitySuppression(behaviour_set);
    it++;
  }
  behaviour_sets_.push_back(behaviour_set);
}
}
