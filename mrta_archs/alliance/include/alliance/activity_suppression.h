#ifndef _ALLIANCE_ACTIVITY_SUPPRESSION_H_
#define _ALLIANCE_ACTIVITY_SUPPRESSION_H_

#include <ros/time.h>
#include <utilities/observer.h>
#include <utilities/functions/unary_sample_holder.h>

namespace alliance
{
class Robot;

class BehaviourSet;

class ActivitySuppression : public utilities::Observer
{
public:
  ActivitySuppression(Robot* robot, BehaviourSet* behaviour_set);
  virtual ~ActivitySuppression();
  virtual void update(utilities::Event *event);
  bool isSuppressed(const ros::Time& timestamp = ros::Time::now()) const;

private:
  Robot* robot_;
  utilities::functions::UnarySampleHolder* suppressed_;
};
}

#endif // _ALLIANCE_ACTIVITY_SUPPRESSION_H_
