#ifndef _ALLIANCE_ROBOT_INTEFACE_H_
#define _ALLIANCE_ROBOT_INTEFACE_H_

#include "alliance/behaviour_set_interface.h"
#include <list>
#include <utilities/has_name.h>

namespace alliance
{
class RobotInterface : public utilities::HasName
{
public:
  RobotInterface(const std::string& id, const std::string& name);
  virtual ~RobotInterface();
  virtual void process() = 0;
  std::list<BehaviourSetInterface*> getBehaviourSets() const;
  Task* getExecutingTask() const;
  bool isIdle() const;

protected:
  BehaviourSetInterface* active_behaviour_set_;
  std::list<BehaviourSetInterface*> behaviour_sets_;
  void addBehaviourSet(BehaviourSetInterface* behaviour_set);
  bool contains(const BehaviourSetInterface& behaviour_set) const;
};
}

#endif // _ALLIANCE_ROBOT_INTEFACE_H_
