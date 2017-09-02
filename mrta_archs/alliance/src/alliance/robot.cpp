#include "alliance/robot.h"

namespace alliance
{
Robot::Robot(const std::string& id, const std::string& name)
    : RobotInterface<Robot, BehaviourSet>::RobotInterface(name, id),
      broadcast_rate_(0.0), timeout_duration_(0.0)
{
}

Robot::~Robot() {}

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
  RobotInterface<Robot, BehaviourSet>::addBehaviourSet(behaviour_set);
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
}
