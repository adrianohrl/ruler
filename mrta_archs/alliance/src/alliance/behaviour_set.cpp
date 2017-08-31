#include "alliance/behaviour_set.h"
#include "alliance/robot.h"
#include <utilities/toggle_event.h>

namespace alliance
{
BehaviourSet::BehaviourSet(Robot* robot, Task* task)
    : Subject::Subject(robot->getId() + "/" + task->getId()), task_(task)
{
  if (!task)
  {
    throw utilities::Exception("The behaviour set's task must not be null.");
  }
  active_ = new utilities::functions::UnarySampleHolder(
      getId() + "/active",
      ros::Duration(10 * robot->getTimeoutDuration().toSec()));
  motivational_behaviour_ = new MotivationalBehaviour(robot, this);
}

BehaviourSet::BehaviourSet(const BehaviourSet& behaviour_set)
    : Subject::Subject(behaviour_set), task_(behaviour_set.task_),
      activation_timestamp_(behaviour_set.activation_timestamp_)
{
  active_ = new utilities::functions::UnarySampleHolder(*behaviour_set.active_);
  motivational_behaviour_ =
      new MotivationalBehaviour(*behaviour_set.motivational_behaviour_);
}

BehaviourSet::~BehaviourSet()
{
  if (active_)
  {
    delete active_;
    active_ = NULL;
  }
  if (motivational_behaviour_)
  {
    delete motivational_behaviour_;
    motivational_behaviour_ = NULL;
  }
  task_ = NULL;
}

void BehaviourSet::process()
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

bool BehaviourSet::isActive(const ros::Time& timestamp) const
{
  return active_->getValue(timestamp);
}

Task* BehaviourSet::getTask() const { return task_; }

ros::Time BehaviourSet::getActivationTimestamp() const
{
  return activation_timestamp_;
}

void BehaviourSet::setActive(bool active, const ros::Time& timestamp)
{
  if (active != active_->getValue(timestamp))
  {
    ROS_DEBUG_STREAM("Updating " << *active_ << " to " << active << ".");
    active_->update(active, timestamp);
    activation_timestamp_ = active ? timestamp : ros::Time();
    utilities::ToggleEvent* event =
        new utilities::ToggleEvent(this, active, timestamp);
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

bool BehaviourSet::operator==(const BehaviourSet& behaviour_set) const
{
  return *task_ == *behaviour_set.task_;
}

bool BehaviourSet::operator!=(const BehaviourSet& behaviour_set) const
{
  return *task_ != *behaviour_set.task_;
}
}
