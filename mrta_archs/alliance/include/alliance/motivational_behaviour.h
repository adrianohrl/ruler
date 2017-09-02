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
  MotivationalBehaviour(Robot* robot, BehaviourSet* behaviour_set);
  virtual ~MotivationalBehaviour();
  bool active(const ros::Time& timestamp = ros::Time::now()) const;
  double getThreshold(const ros::Time& timestamp = ros::Time::now()) const;
  double getLevel(const ros::Time& timestamp = ros::Time::now()) const;
  ActivitySuppression* getActivitySuppression() const;
  InterCommunication* getInterCommunication() const;
  void setThreshold(double threshold,
                    const ros::Time& timestamp = ros::Time::now());
  void setImpatience(double fast_rate,
                     const ros::Time& timestamp = ros::Time::now());
  void setAcquiescence(const ros::Duration& yielding_delay,
                       const ros::Duration& giving_up_delay,
                       const ros::Time& timestamp = ros::Time::now());

private:
  utilities::functions::ContinuousSampleHolder* threshold_;
  utilities::functions::ContinuousSampleHolder* motivation_;
  Acquiescence* acquiescence_;
  ActivitySuppression* activity_suppression_;
  Impatience* impatience_;
  ImpatienceReset* impatience_reset_;
  InterCommunication* inter_communication_;
  SensoryFeedback* sensory_feedback_;
  Robot* robot_;
};
}

#endif // _ALLIANCE_MOTIVATIONAL_BEHAVIOUR_H_
