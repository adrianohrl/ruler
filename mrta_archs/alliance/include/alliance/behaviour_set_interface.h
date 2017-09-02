#ifndef _ALLIANCE_BEHAVIOUR_SET_INTERFACE_H_
#define _ALLIANCE_BEHAVIOUR_SET_INTERFACE_H_

#include "alliance/task.h"
#include <utilities/subject.h>
#include <utilities/functions/unary_sample_holder.h>

namespace alliance
{
template <typename R>
class BehaviourSetInterface
{
public:
  BehaviourSetInterface(R* robot, Task* task);
  virtual ~BehaviourSetInterface();
  virtual void process() = 0;
  Task* getTask() const;

protected:
  R* robot_;
  Task* task_;
};

template <typename R>
BehaviourSetInterface<R>::BehaviourSetInterface(R* robot, Task* task)
    : robot_(robot), task_(task)
{
}

template <typename R>
BehaviourSetInterface<R>::~BehaviourSetInterface()
{
  robot_ = NULL;
  task_ = NULL;
}

template <typename R>
Task* BehaviourSetInterface<R>::getTask() const { return task_; }
}
#endif // _ALLIANCE_BEHAVIOUR_SET_INTERFACE_H_
