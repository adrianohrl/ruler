#include "alliance/acquiescence.h"
#include "alliance/robot.h"

namespace alliance
{
Acquiescence::Acquiescence(Robot* robot, BehaviourSet* behaviour_set,
                           InterCommunication* monitor)
    : robot_(robot), monitor_(monitor)
{
  yielding_delay_ = new utilities::functions::ContinuousSampleHolder(
      behaviour_set->getId() + "/acquiescence/yielding_delay", 0.0,
      ros::Duration(10 * robot_->getTimeoutDuration().toSec()));
  giving_up_delay_ = new utilities::functions::ContinuousSampleHolder(
      behaviour_set->getId() + "/acquiescence/giving_up_delay", 0.0,
      ros::Duration(10 * robot_->getTimeoutDuration().toSec()));
}

Acquiescence::Acquiescence(const Acquiescence& acquiescence)
{

  yielding_delay_ = new utilities::functions::ContinuousSampleHolder(
      *acquiescence.yielding_delay_);
  giving_up_delay_ = new utilities::functions::ContinuousSampleHolder(
      *acquiescence.giving_up_delay_);
}

Acquiescence::~Acquiescence()
{
  if (yielding_delay_)
  {
    delete yielding_delay_;
    yielding_delay_ = NULL;
  }
  if (giving_up_delay_)
  {
    delete giving_up_delay_;
    giving_up_delay_ = NULL;
  }
  robot_ = NULL;
  monitor_ = NULL;
}

ros::Duration Acquiescence::getYieldingDelay(const ros::Time& timestamp) const
{
  return ros::Duration(yielding_delay_->getValue(timestamp));
}

ros::Duration Acquiescence::getGivingUpDelay(const ros::Time& timestamp) const
{
  return ros::Duration(giving_up_delay_->getValue(timestamp));
}

bool Acquiescence::isAcquiescent(const ros::Time& timestamp)
{
  return false;
}

void Acquiescence::setYieldingDelay(const ros::Duration& yielding_delay,
                                    const ros::Time& timestamp)
{
  yielding_delay_->update(yielding_delay.toSec(), timestamp);
}

void Acquiescence::setGivingUpDelay(const ros::Duration& giving_up_delay,
                                    const ros::Time& timestamp)
{
  giving_up_delay_->update(giving_up_delay.toSec(), timestamp);
}
}
