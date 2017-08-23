#include "alliance/robot.h"

namespace alliance
{
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

void Robot::setBroadcastRate(double broadcast_rate)
{
  broadcast_rate_ = fabs(broadcast_rate);
}

void Robot::setQuietDuration(ros::Duration quiet_duration)
{
  quiet_duration_ = quiet_duration;
}
}
