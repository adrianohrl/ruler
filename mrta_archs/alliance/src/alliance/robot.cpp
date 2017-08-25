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

void Robot::process()
{
  std::list<BehaviourSet*>::iterator it(behaviour_sets_.begin());
  while (it != behaviour_sets_.end())
  {
    BehaviourSet* behaviour_set = *it;
    behaviour_set->process();
    if (behaviour_set->isActive())
    {
      // mas e se houver mais de um comportamento ativo ????
      active_behaviour_set_ = behaviour_set;
      return;
    }
    it++;
  }
}

bool Robot::isActive() const
{
  return active_behaviour_set_ && active_behaviour_set_->isActive();
}

ros::Rate Robot::getBroadcastRate() const { return broadcast_rate_; }

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

void Robot::setBroadcastRate(ros::Rate broadcast_rate)
{
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
  if (!behaviour_set)
  {
    ROS_ERROR_STREAM("The given robot's behaviour set must not be null.");
    return;
  }
  if (contains(*behaviour_set))
  {
    ROS_WARN_STREAM(*this << " robot already have the " << *behaviour_set
                          << " behaviour set.");
    return;
  }
  std::list<BehaviourSet*>::iterator it(behaviour_sets_.begin());
  while (it != behaviour_sets_.end())
  {
    BehaviourSet* robot_behaviour_set = *it;
    behaviour_set->registerActivitySuppression(robot_behaviour_set);
    robot_behaviour_set->registerActivitySuppression(behaviour_set);
    it++;
  }
  behaviour_sets_.push_back(behaviour_set);
}

bool Robot::contains(const BehaviourSet& behaviour_set) const
{
  std::list<BehaviourSet*>::const_iterator it(behaviour_sets_.begin());
  while (it != behaviour_sets_.end())
  {
    if (**it == behaviour_set)
    {
      return true;
    }
    it++;
  }
  return false;
}
}
