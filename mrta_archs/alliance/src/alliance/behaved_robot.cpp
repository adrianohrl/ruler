#include "alliance/behaved_robot.h"

namespace alliance
{
BehavedRobot::BehavedRobot(const std::string& id, const std::string& name)
    : RobotInterface<BehavedRobot, LayeredBehaviourSet>::RobotInterface(id,
                                                                        name)
{
}

BehavedRobot::~BehavedRobot() {}

void BehavedRobot::update(const alliance_msgs::BeaconSignal& msg) {}
}
