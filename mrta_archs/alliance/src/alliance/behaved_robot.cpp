#include "alliance/behaved_robot.h"

namespace alliance
{
BehavedRobot::BehavedRobot(const std::string& id, const std::string& name)
    : RobotInterface::RobotInterface(id, name)
{
}

BehavedRobot::~BehavedRobot() {}

void BehavedRobot::process()
{
  if (active_behaviour_set_)
  {
    active_behaviour_set_->process();
  }
}

void BehavedRobot::addBehaviourSet(LayeredBehaviourSet* behaviour_set)
{
  RobotInterface::addBehaviourSet(behaviour_set);
}
}
