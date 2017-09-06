#ifndef _ALLIANCE_MOTIVATIONAL_BEHAVIOUR_H_
#define _ALLIANCE_MOTIVATIONAL_BEHAVIOUR_H_

#include <ros/time.h>
#include "alliance/acquiescence.h"
#include "alliance/activity_suppression.h"
#include "alliance/impatience.h"
#include "alliance/impatience_reset.h"
#include "alliance/inter_communication.h"
#include "alliance/sensory_feedback.h"
#include <utilities/functions/continuous_sample_holder.h>

namespace alliance
{
class MotivationalBehaviour
{
public:
  MotivationalBehaviour(const RobotPtr& robot, const BehaviourSetPtr& behaviour_set);
  virtual ~MotivationalBehaviour();
  bool active(const ros::Time& timestamp = ros::Time::now()) const;
  double getThreshold(const ros::Time& timestamp = ros::Time::now()) const;
  double getLevel(const ros::Time& timestamp = ros::Time::now()) const;
  ActivitySuppressionPtr getActivitySuppression() const;
  InterCommunicationPtr getInterCommunication() const;
  void setThreshold(double threshold,
                    const ros::Time& timestamp = ros::Time::now());
  void setImpatience(double fast_rate,
                     const ros::Time& timestamp = ros::Time::now());
  void setAcquiescence(const ros::Duration& yielding_delay,
                       const ros::Duration& giving_up_delay,
                       const ros::Time& timestamp = ros::Time::now());

private:
  const RobotPtr robot_;
  const AcquiescencePtr acquiescence_;
  const ActivitySuppressionPtr activity_suppression_;
  const ImpatiencePtr impatience_;
  const ImpatienceResetPtr impatience_reset_;
  const InterCommunicationPtr inter_communication_;
  const SensoryFeedbackPtr sensory_feedback_;
  const utilities::functions::ContinuousSampleHolderPtr threshold_;
  const utilities::functions::ContinuousSampleHolderPtr motivation_;
};

typedef boost::shared_ptr<MotivationalBehaviour> MotivationalBehaviourPtr;
typedef boost::shared_ptr<MotivationalBehaviour const> MotivationalBehaviourConstPtr;
}

#endif // _ALLIANCE_MOTIVATIONAL_BEHAVIOUR_H_
