#include "alliance/behaved_robot.h"

namespace alliance
{
BehavedRobot::BehavedRobot(const std::string& id, const std::string& name,
                           const std::string& ns)
    : RobotInterface<LayeredBehaviourSet>::RobotInterface(id, name, ns)
{
}

BehavedRobot::~BehavedRobot() {}
}
