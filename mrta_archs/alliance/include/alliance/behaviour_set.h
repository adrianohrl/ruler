#ifndef _ALLIANCE_BEHAVIOUR_SET_H_
#define _ALLIANCE_BEHAVIOUR_SET_H_

#include "alliance/motivational_behaviour.h"
#include "alliance/task.h"
#include <utilities/subject.h>

namespace alliance
{
class BehaviourSet //: public utilities::Subject
{
public:
  BehaviourSet(const BehaviourSet& behaviour_set);
  virtual ~BehaviourSet();
  MotivationalBehaviour* getMotivationalBehaviour() const;
  bool isActive() const;
  Task* getTask() const;
  void setActive(bool active = true);

private:
  bool active_;
  MotivationalBehaviour* motivational_behaviour_;
  Task* task_;
};
}

#endif // _ALLIANCE_BEHAVIOUR_SET_H_
