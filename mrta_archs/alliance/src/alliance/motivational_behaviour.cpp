#include "alliance/motivational_behaviour.h"
#include "alliance/robot.h"

namespace alliance
{
MotivationalBehaviour::MotivationalBehaviour(
    const MotivationalBehaviour& motivational_behaviour)
    : threshold_(motivational_behaviour.threshold_),
      activity_suppression_(motivational_behaviour.activity_suppression_),
      impatience_reset_(motivational_behaviour.impatience_reset_),
      inter_communication_(motivational_behaviour.inter_communication_),
      sensory_feedback_(motivational_behaviour.sensory_feedback_),
      robot_(motivational_behaviour.robot_)
{
}

MotivationalBehaviour::~MotivationalBehaviour()
{
  if (activity_suppression_)
  {
    delete activity_suppression_;
    activity_suppression_ = NULL;
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

double MotivationalBehaviour::getThreshold() const { return threshold_; }

ActivitySuppression* MotivationalBehaviour::getActivitySuppression() const
{
  return activity_suppression_;
}

ImpatienceReset* MotivationalBehaviour::getImpatienceReset() const
{
  return impatience_reset_;
}

InterCommunication* MotivationalBehaviour::getInterCommunication() const
{
  return inter_communication_;
}

SensoryFeedback* MotivationalBehaviour::getSensoryFeedback() const
{
  return sensory_feedback_;
}

Robot *MotivationalBehaviour::getRobot() const
{
  return robot_;
}

double MotivationalBehaviour::getLevel(ros::Time timestamp) const
{
  return (0.0 + robot_->getImpatience(timestamp))
      * activity_suppression_->suppress(timestamp)
      * impatience_reset_->reset(timestamp)
      * inter_communication_->received(timestamp)
      * sensory_feedback_->received(timestamp)
      * robot_->isAcquiescent(timestamp);
}
}
