#include "alliance/motivational_behaviour.h"
#include "alliance/behaviour_set.h"
#include "alliance/robot.h"

namespace alliance
{
MotivationalBehaviour::MotivationalBehaviour(
    const RobotPtr& robot, const BehaviourSetPtr& behaviour_set)
    : robot_(robot), behaviour_set_(behaviour_set),
      threshold_(new SampleHolder(behaviour_set->getId() + "/threshold", 0.0,
                                  behaviour_set_->getBufferHorizon())),
      motivation_(new SampleHolder(behaviour_set->getId() + "/motivation", 0.0,
                                   behaviour_set_->getBufferHorizon())),
      monitor_(new InterRobotCommunication(robot, behaviour_set)),
      acquiescence_(new Acquiescence(robot, behaviour_set)),
      activity_suppression_(new ActivitySuppression(robot, behaviour_set)),
      impatience_(new Impatience(robot, behaviour_set)),
      impatience_reset_(new ImpatienceReset()),
      sensory_feedback_(new SensoryFeedback(robot, behaviour_set))
{
}

MotivationalBehaviour::~MotivationalBehaviour() {}

void MotivationalBehaviour::init()
{
  acquiescence_->init(monitor_);
  impatience_->init(monitor_);
  impatience_reset_->init(monitor_);
}

bool MotivationalBehaviour::isActive(const ros::Time& timestamp) const
{
  return getLevel(timestamp) >= getThreshold(timestamp);
}

double MotivationalBehaviour::getThreshold(const ros::Time& timestamp) const
{
  return threshold_->getValue(timestamp);
}

double MotivationalBehaviour::getLevel(const ros::Time& timestamp) const
{
  double motivation(motivation_->getValue(timestamp));
  /*double impatience(impatience_->getLevel(timestamp));
  bool acquiescent(acquiescence_->isAcquiescent(timestamp));
  bool suppressed(activity_suppression_->isSuppressed(timestamp));
  bool resetted(impatience_reset_->isResetted(timestamp));
  bool applicable(sensory_feedback_->isApplicable(timestamp));
  ROS_WARN_STREAM("[Motivation] "
                  << *behaviour_set_ << " m0: " << motivation
                  << ", ipt: " << impatience << ", acq: " << acquiescent
                  << ", sup: " << suppressed << ", res: " << resetted
                  << ", app: " << applicable);*/
  ROS_WARN_STREAM("[MotivationalBehaviour]" << *sensory_feedback_ << ": "
                  << (sensory_feedback_->isApplicable(timestamp) ? "true": "false"));
  motivation = (motivation + impatience_->getLevel(timestamp)) *
               !acquiescence_->isAcquiescent(timestamp) *
               !activity_suppression_->isSuppressed(timestamp) *
               !impatience_reset_->isResetted(timestamp) *
               sensory_feedback_->isApplicable(timestamp);
  motivation_->update(motivation, timestamp);
  return motivation;
}

ActivitySuppressionPtr MotivationalBehaviour::getActivitySuppression() const
{
  return activity_suppression_;
}

ImpatiencePtr MotivationalBehaviour::getImpatience() const
{
  return impatience_;
}

ImpatienceResetPtr MotivationalBehaviour::getImpatienceReset() const
{
  return impatience_reset_;
}

InterRobotCommunicationPtr
MotivationalBehaviour::getInterRobotCommunication() const
{
  return monitor_;
}

SensoryFeedbackPtr MotivationalBehaviour::getSensoryFeedback() const
{
  return sensory_feedback_;
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
