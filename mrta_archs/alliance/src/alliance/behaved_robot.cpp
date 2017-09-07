#include "alliance/behaved_robot.h"

namespace alliance
{
BehavedRobot::BehavedRobot(const std::string& id, const std::string& name)
    : RobotInterface<LayeredBehaviourSet>::RobotInterface(id, name)
{
}

BehavedRobot::~BehavedRobot() {}
}
