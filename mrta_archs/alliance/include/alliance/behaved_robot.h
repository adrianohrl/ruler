#ifndef _ALLIANCE_BEHAVED_ROBOT_H_
#define _ALLIANCE_BEHAVED_ROBOT_H_

#include "alliance/layered_behaviour_set.h"
#include "alliance/robot_interface.h"
#include <alliance_msgs/BeaconSignal.h>

namespace alliance
{
class BehavedRobot : public RobotInterface<BehavedRobot, LayeredBehaviourSet>
{
public:
  BehavedRobot(const std::string& id, const std::string& name);
  virtual ~BehavedRobot();
  void update(const alliance_msgs::BeaconSignal& msg);
};
}

#endif // _ALLIANCE_BEHAVED_ROBOT_H_
