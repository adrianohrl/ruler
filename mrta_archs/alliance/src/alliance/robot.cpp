#include "alliance/robot.h"

namespace alliance
{
Robot::Robot(std::string id, std::string name)
    : HasName::HasName(name, id), broadcast_rate_(0.0), quiet_duration_(0.0),
      acquiescence_(NULL), active_behaviour_set_(NULL), impatience_(NULL)
{
}

Robot::Robot(const Robot& robot)
    : HasName::HasName(robot), broadcast_rate_(robot.broadcast_rate_),
      quiet_duration_(robot.quiet_duration_),
      acquiescence_(robot.acquiescence_),
      active_behaviour_set_(robot.active_behaviour_set_),
      behaviour_sets_(robot.behaviour_sets_), impatience_(robot.impatience_)
{
}

Robot::~Robot()
{
  if (acquiescence_)
  {
    delete acquiescence_;
    acquiescence_ = NULL;
  }
  active_behaviour_set_ = NULL;
  std::list<BehaviourSet*>::iterator it(behaviour_sets_.begin());
  while (it != behaviour_sets_.end())
  {
    if (*it)
    {
      delete *it;
      *it = NULL;
    }
    it++;
  }
  if (impatience_)
  {
    delete impatience_;
    impatience_ = NULL;
  }
}

bool Robot::isActive() const
{
  return active_behaviour_set_ && active_behaviour_set_->isActive();
}

double Robot::getBroadcastRate() const { return broadcast_rate_; }

ros::Duration Robot::getQuietDuration() const { return quiet_duration_; }

double Robot::getImpatience(ros::Time timestamp) const
{
  return impatience_->getLevel(timestamp);
}

bool Robot::isAcquiescent(ros::Time timestamp) const
{
  return acquiescence_->isAcquiescent(timestamp);
}

std::list<BehaviourSet*> Robot::getBehaviourSets() const
{
  return behaviour_sets_;
}

Task* Robot::getExecutingTask() const
{
  return active_behaviour_set_ ? active_behaviour_set_->getTask() : NULL;
}

void Robot::setBroadcastRate(double broadcast_rate)
{
  if (broadcast_rate <= 0.0)
  {
    throw utilities::Exception(
        "The robot's inter communication broadcast rate must be positive.");
  }
  broadcast_rate_ = broadcast_rate;
}

void Robot::setQuietDuration(ros::Duration quiet_duration)
{
  quiet_duration_ = quiet_duration;
}

void Robot::setAcquiescence(ros::Duration yielding_delay,
                            ros::Duration giving_up_delay)
{
  if (!acquiescence_)
  {
    acquiescence_ = new Acquiescence(yielding_delay, giving_up_delay);
    return;
  }
  acquiescence_->setYieldingDelay(yielding_delay);
  acquiescence_->setGivingUpDelay(giving_up_delay);
}

void Robot::setImpatience(double fast_rate)
{
  if (!impatience_)
  {
    impatience_ = new Impatience(fast_rate);
    return;
  }
  impatience_->setFastRate(fast_rate);
}

void Robot::addBehaviourSet(BehaviourSet* behaviour_set)
{
  if (behaviour_set)
  {
    behaviour_sets_.push_back(behaviour_set);
  }
  std::list<BehaviourSet*>::iterator it(behaviour_sets_.begin());
  while (it != behaviour_sets_.end())
  {
    behaviour_set->registerActivitySuppression(*it);
    it++;
  }
}
}
