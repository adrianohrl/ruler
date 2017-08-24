#include "alliance/behaviour_set.h"
#include "alliance/motivational_behaviour.h"
#include "alliance/robot.h"

namespace alliance
{
MotivationalBehaviour::MotivationalBehaviour(Robot* robot,
                                             BehaviourSet* behaviour_set)
    : robot_(robot), threshold_(0.0),
      activity_suppression_(new ActivitySuppression(robot, behaviour_set)),
      impatience_reset_(new ImpatienceReset(robot)),
      inter_communication_(new InterCommunication(robot, behaviour_set)),
      sensory_feedback_(new SensoryFeedback())
{
}

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

bool MotivationalBehaviour::active(ros::Time timestamp) const
{
  return getLevel(timestamp) >= threshold_;
}

double MotivationalBehaviour::getLevel(ros::Time timestamp) const
{
  return (0.0 + robot_->getImpatience(timestamp)) *
         activity_suppression_->suppress(timestamp) *
         impatience_reset_->reset(timestamp) *
         inter_communication_->received(timestamp) *
         sensory_feedback_->received(timestamp) *
         robot_->isAcquiescent(timestamp);
}

ActivitySuppression* MotivationalBehaviour::getActivitySuppression() const
{
  return activity_suppression_;
}

InterCommunication* MotivationalBehaviour::getInterCommunication() const
{
  return inter_communication_;
}

void MotivationalBehaviour::setThreshold(double threshold)
{
  if (threshold <= 0.0)
  {
    throw utilities::Exception(
        "The motivational behaviour's activation threshold must be positive.");
  }
  threshold_ = threshold;
}
}
