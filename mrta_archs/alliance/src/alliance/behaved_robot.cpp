#include "alliance/behaved_robot.h"

namespace alliance
{
BehavedRobot::BehavedRobot(const std::string& id, const std::string& name)
    : RobotInterface<BehavedRobot, LayeredBehaviourSet>::RobotInterface(id,
                                                                        name)
{
}

BehavedRobot::~BehavedRobot() {}

void BehavedRobot::process()
{
  if (active_behaviour_set_)
  {
    ROS_WARN_STREAM("[BehavedRobot] active: " << *active_behaviour_set_);
    active_behaviour_set_->process();
  }
}

void BehavedRobot::update(const alliance_msgs::BeaconSignal& msg) {}
}
