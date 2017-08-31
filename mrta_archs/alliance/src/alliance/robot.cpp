#include "alliance/robot.h"

namespace alliance
{
Robot::Robot(const std::string& id, const std::string& name)
    : HasName::HasName(name, id), broadcast_rate_(0.0), timeout_duration_(0.0),
      active_behaviour_set_(NULL)
{
}

Robot::Robot(const Robot& robot)
    : HasName::HasName(robot), broadcast_rate_(robot.broadcast_rate_),
      timeout_duration_(robot.timeout_duration_),
      active_behaviour_set_(robot.active_behaviour_set_),
      behaviour_sets_(robot.behaviour_sets_)
{
}

Robot::~Robot()
{
  active_behaviour_set_ = NULL;
  std::list<BehaviourSet*>::iterator it(behaviour_sets_.begin());
  while (it != behaviour_sets_.end())
  {
    if (*it)
    {
      delete *it;
      *it = NULL;
    }
    it++;
  }
}

void Robot::process()
{
  std::list<BehaviourSet*>::iterator it(behaviour_sets_.begin());
  while (it != behaviour_sets_.end())
  {
    BehaviourSet* behaviour_set = *it;
    behaviour_set->process();
    if (behaviour_set->isActive())
    {
      if (active_behaviour_set_ && *active_behaviour_set_ != *behaviour_set)
      {
        active_behaviour_set_->setActive(false);
      }
      active_behaviour_set_ = behaviour_set;
      active_behaviour_set_->setActive();
      return;
    }
    it++;
  }
}

bool Robot::isActive() const
{
  return active_behaviour_set_ && active_behaviour_set_->isActive();
}

ros::Rate Robot::getBroadcastRate() const { return broadcast_rate_; }

ros::Duration Robot::getTimeoutDuration() const { return timeout_duration_; }

std::list<BehaviourSet*> Robot::getBehaviourSets() const
{
  return behaviour_sets_;
}

Task* Robot::getExecutingTask() const
{
  return active_behaviour_set_ ? active_behaviour_set_->getTask() : NULL;
}

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
  std::list<BehaviourSet*>::iterator it(behaviour_sets_.begin());
  while (it != behaviour_sets_.end())
  {
    BehaviourSet* robot_behaviour_set = *it;
    behaviour_set->registerActivitySuppression(robot_behaviour_set);
    robot_behaviour_set->registerActivitySuppression(behaviour_set);
    it++;
  }
  behaviour_sets_.push_back(behaviour_set);
}

bool Robot::contains(const BehaviourSet& behaviour_set) const
{
  std::list<BehaviourSet*>::const_iterator it(behaviour_sets_.begin());
  while (it != behaviour_sets_.end())
  {
    if (**it == behaviour_set)
    {
      return true;
    }
    it++;
  }
  return false;
}
}
