#include "alliance/behaviour_set.h"
#include "alliance/robot.h"
#include <utilities/toggle_event.h>

namespace alliance
{
BehaviourSet::BehaviourSet(Robot* robot, Task* task)
    : Subject::Subject(robot->getId() + "/" + task->getId()), task_(task),
      motivational_behaviour_(new MotivationalBehaviour(robot))
{
}

BehaviourSet::BehaviourSet(const BehaviourSet& behaviour_set)
    : Subject::Subject(behaviour_set),
      motivational_behaviour_(behaviour_set.motivational_behaviour_),
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
    active_ = active;
    utilities::ToggleEvent* event = new utilities::ToggleEvent(this, active);
    Subject::notify(event);
    delete event;
  }
}

void BehaviourSet::setActivationThreshold(double threshold)
{
  if (!motivational_behaviour_)
  {
    throw utilities::Exception("The motivational behaviour of the " + str() +
                               " behaviour set has not been initialized yet.");
  }
  motivational_behaviour_->setThreshold(threshold);
}

void BehaviourSet::registerActivitySuppression(BehaviourSet* behaviour_set)
{
  Subject::registerObserver(
      behaviour_set->motivational_behaviour_->getActivitySuppression());
}
}
