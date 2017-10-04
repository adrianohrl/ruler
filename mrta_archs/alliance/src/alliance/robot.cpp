#include "alliance/robot.h"

namespace alliance
{
Robot::Robot(const std::string& id, const std::string& name,
             const std::string& ns)
    : RobotInterface<BehaviourSet>::RobotInterface(id, name, ns),
      broadcast_rate_(0.0), timeout_duration_(0.0)
{
}

Robot::~Robot() {}

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

void Robot::addBehaviourSet(const BehaviourSetPtr& behaviour_set)
{
  RobotInterface<BehaviourSet>::addBehaviourSet(behaviour_set);
  std::list<BehaviourSetPtr>::iterator it(behaviour_sets_.begin());
  while (it != behaviour_sets_.end())
  {
    BehaviourSetPtr robot_behaviour_set(*it);
    behaviour_set->registerActivitySuppression(robot_behaviour_set);
    robot_behaviour_set->registerActivitySuppression(behaviour_set);
    it++;
  }
  behaviour_sets_.push_back(behaviour_set);
}
}
