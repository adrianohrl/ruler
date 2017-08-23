#include "alliance/behaviour_set.h"

namespace alliance
{
BehaviourSet::BehaviourSet(const BehaviourSet& behaviour_set)
    : motivational_behaviour_(behaviour_set.motivational_behaviour_),
      task_(behaviour_set.task_)
{
}

BehaviourSet::~BehaviourSet()
{
  if (motivational_behaviour_)
  {
    delete motivational_behaviour_;
    motivational_behaviour_ = NULL;
  }
  task_ = NULL;
}

MotivationalBehaviour* BehaviourSet::getMotivationalBehaviour() const
{
  return motivational_behaviour_;
}

bool BehaviourSet::isActive() const { return active_; }

Task* BehaviourSet::getTask() const { return task_; }

void BehaviourSet::setActive(bool active)
{
  if (active != active_)
  {
    //Subject::notify(active);
    active_ = active;
  }
}
}
