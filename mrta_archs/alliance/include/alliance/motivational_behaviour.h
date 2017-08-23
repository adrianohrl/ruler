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

class MotivationalBehaviour
{
public:
  MotivationalBehaviour(const MotivationalBehaviour& motivational_behaviour);
  virtual ~MotivationalBehaviour();
  double getThreshold() const;
  ActivitySuppression* getActivitySuppression() const;
  ImpatienceReset* getImpatienceReset() const;
  InterCommunication* getInterCommunication() const;
  SensoryFeedback* getSensoryFeedback() const;
  Robot* getRobot() const;
  double getLevel(ros::Time timestamp = ros::Time::now()) const;

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
