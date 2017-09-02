#include "alliance/behaviour_set.h"
#include "alliance/motivational_behaviour.h"
#include "alliance/robot.h"

namespace alliance
{
MotivationalBehaviour::MotivationalBehaviour(Robot* robot,
                                             BehaviourSet* behaviour_set)
    : robot_(robot)
{
  threshold_ = new utilities::functions::ContinuousSampleHolder(
      behaviour_set->getId() + "/threshold", 0.0,
      ros::Duration(10 * robot_->getTimeoutDuration().toSec()));
  motivation_ = new utilities::functions::ContinuousSampleHolder(
      behaviour_set->getId() + "/motivation", 0.0,
      ros::Duration(10 * robot_->getTimeoutDuration().toSec()));
  inter_communication_ = new InterCommunication(robot, behaviour_set);
  acquiescence_ = new Acquiescence(robot, behaviour_set, inter_communication_);
  activity_suppression_ = new ActivitySuppression(robot, behaviour_set);
  impatience_ = new Impatience(robot, behaviour_set, inter_communication_);
  impatience_reset_ = new ImpatienceReset(inter_communication_);
  sensory_feedback_ = new SensoryFeedback(behaviour_set->getTask());
}

MotivationalBehaviour::~MotivationalBehaviour()
{
  if (threshold_)
  {
    delete threshold_;
    threshold_ = NULL;
  }
  if (motivation_)
  {
    delete motivation_;
    motivation_ = NULL;
  }
  if (acquiescence_)
  {
    delete acquiescence_;
    acquiescence_ = NULL;
  }
  if (activity_suppression_)
  {
    delete activity_suppression_;
    activity_suppression_ = NULL;
  }
  if (impatience_)
  {
    delete impatience_;
    impatience_ = NULL;
  }
  if (impatience_reset_)
  {
    delete impatience_reset_;
    impatience_reset_ = NULL;
  }
  if (inter_communication_)
  {
    delete inter_communication_;
    inter_communication_ = NULL;
  }
  robot_ = NULL;
}

bool MotivationalBehaviour::active(const ros::Time& timestamp) const
{
  return getLevel(timestamp) >= threshold_->getValue(timestamp);
}

double MotivationalBehaviour::getThreshold(const ros::Time& timestamp) const
{
  return threshold_->getValue(timestamp);
}

double MotivationalBehaviour::getLevel(const ros::Time& timestamp) const
{
  double motivation(motivation_->getValue());
  motivation = (motivation + impatience_->getLevel(timestamp)) *
               acquiescence_->isAcquiescent(timestamp) *
               activity_suppression_->isSuppressed(timestamp) *
               impatience_reset_->isResetted(timestamp) *
               sensory_feedback_->isApplicable(timestamp);
  motivation_->update(motivation, timestamp);
  return motivation;
}

ActivitySuppression* MotivationalBehaviour::getActivitySuppression() const
{
  return activity_suppression_;
}

InterCommunication* MotivationalBehaviour::getInterCommunication() const
{
  return inter_communication_;
}

void MotivationalBehaviour::setThreshold(double threshold,
                                         const ros::Time& timestamp)
{
  if (threshold <= 0.0)
  {
    throw utilities::Exception(
        "The motivational behaviour's activation threshold must be positive.");
  }
  ROS_DEBUG_STREAM("Updating " << *threshold_ << " to " << threshold << ".");
  threshold_->update(threshold, timestamp);
}

void MotivationalBehaviour::setImpatience(double fast_rate,
                                          const ros::Time& timestamp)
{
  impatience_->setFastRate(fast_rate, timestamp);
}

void MotivationalBehaviour::setAcquiescence(
    const ros::Duration& yielding_delay, const ros::Duration& giving_up_delay,
    const ros::Time& timestamp)
{
  acquiescence_->setYieldingDelay(yielding_delay, timestamp);
  acquiescence_->setGivingUpDelay(giving_up_delay, timestamp);
}
}
