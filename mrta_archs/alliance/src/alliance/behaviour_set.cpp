#include "alliance/behaviour_set.h"
#include "alliance/robot.h"
#include <utilities/toggle_event.h>

namespace alliance
{
BehaviourSet::BehaviourSet(Robot* robot, Task* task,
                           ros::Duration buffer_horizon)
    : BehaviourSetInterface<Robot>::BehaviourSetInterface(robot, task,
                                                          buffer_horizon),
      Subject::Subject(robot->getId() + "/" + task->getId())
{
  motivational_behaviour_ = new MotivationalBehaviour(robot, this);
}

BehaviourSet::~BehaviourSet()
{
  if (motivational_behaviour_)
  {
    delete motivational_behaviour_;
    motivational_behaviour_ = NULL;
  }
}

void BehaviourSet::preProcess()
{
  /** so para testar agora **/
  std::string id(getId());
  return setActive(id == "robot1/wander" || id == "robot2/report" ||
                   id == "robot3/border_protection");
  /* acaba aki o test */

  // setActive(motivational_behaviour_->active());
}

MotivationalBehaviour* BehaviourSet::getMotivationalBehaviour() const
{
  return motivational_behaviour_;
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

void BehaviourSet::setAcquiescence(const ros::Duration& yielding_delay,
                                   const ros::Duration& giving_up_delay)
{
  motivational_behaviour_->setAcquiescence(yielding_delay, giving_up_delay);
}

void BehaviourSet::setImpatience(double fast_rate)
{
  motivational_behaviour_->setImpatience(fast_rate);
}

void BehaviourSet::registerActivitySuppression(BehaviourSet* behaviour_set)
{
  Subject::registerObserver(
      behaviour_set->motivational_behaviour_->getActivitySuppression());
}

void BehaviourSet::setActive(bool active, const ros::Time& timestamp)
{
  if (active != isActive(timestamp))
  {
    BehaviourSetInterface<Robot>::setActive(active, timestamp);
    utilities::ToggleEvent* event =
        new utilities::ToggleEvent(this, active, timestamp);
    Subject::notify(event);
    delete event;
  }
}
}
