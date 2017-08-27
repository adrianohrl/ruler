#ifndef _ALLIANCE_MOTIVATIONAL_BEHAVIOUR_H_
#define _ALLIANCE_MOTIVATIONAL_BEHAVIOUR_H_

#include <ros/time.h>
#include "alliance/activity_suppression.h"
#include "alliance/impatience_reset.h"
#include "alliance/inter_communication.h"
#include "alliance/sensory_feedback.h"

namespace alliance
{
class Robot;

class BehaviourSet;

class MotivationalBehaviour
{
public:
  MotivationalBehaviour(Robot* robot, BehaviourSet* behaviour_set);
  MotivationalBehaviour(const MotivationalBehaviour& motivational_behaviour);
  virtual ~MotivationalBehaviour();
  bool active(const ros::Time& timestamp = ros::Time::now()) const;
  double getLevel(const ros::Time& timestamp = ros::Time::now()) const;
  ActivitySuppression* getActivitySuppression() const;
  InterCommunication* getInterCommunication() const;
  void setThreshold(double threshold);

private:
  double threshold_;
  ActivitySuppression* activity_suppression_;
  ImpatienceReset* impatience_reset_;
  InterCommunication* inter_communication_;
  SensoryFeedback* sensory_feedback_;
  Robot* robot_;
};
}

#endif // _ALLIANCE_MOTIVATIONAL_BEHAVIOUR_H_
