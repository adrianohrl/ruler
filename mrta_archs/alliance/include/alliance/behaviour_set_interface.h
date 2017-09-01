#ifndef _ALLIANCE_BEHAVIOUR_SET_INTERFACE_H_
#define _ALLIANCE_BEHAVIOUR_SET_INTERFACE_H_

#include "alliance/task.h"
#include <utilities/subject.h>
#include <utilities/functions/unary_sample_holder.h>

namespace alliance
{
class RobotInterface;

class BehaviourSetInterface : public utilities::Subject
{
public:
  BehaviourSetInterface(RobotInterface* robot, Task* task);
  virtual ~BehaviourSetInterface();
  virtual void process() = 0;
  Task* getTask() const;

private:
  Task* task_;
};
}

#endif // _ALLIANCE_BEHAVIOUR_SET_INTERFACE_H_
