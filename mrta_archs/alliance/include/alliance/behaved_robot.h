#ifndef _ALLIANCE_BEHAVED_ROBOT_H_
#define _ALLIANCE_BEHAVED_ROBOT_H_

#include "alliance/layered_behaviour_set.h"
#include "alliance/robot_interface.h"

namespace alliance
{
class BehavedRobot : public RobotInterface
{
public:
  BehavedRobot(const std::string& id, const std::string& name);
  virtual ~BehavedRobot();
  virtual void process();
  void addBehaviourSet(LayeredBehaviourSet* behaviour_set);
};
}

#endif // _ALLIANCE_BEHAVED_ROBOT_H_
