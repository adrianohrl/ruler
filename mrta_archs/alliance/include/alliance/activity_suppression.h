#ifndef _ALLIANCE_ACTIVITY_SUPPRESSION_H_
#define _ALLIANCE_ACTIVITY_SUPPRESSION_H_

#include <ros/time.h>
#include <utilities/observer.h>

namespace alliance
{
class Robot;

class BehaviourSet;

class ActivitySuppression : public utilities::Observer
{
public:
  ActivitySuppression(Robot* robot, BehaviourSet* behaviour_set);
  ActivitySuppression(const ActivitySuppression& activity_suppression);
  virtual ~ActivitySuppression();
  virtual void update(utilities::Event* event);
  bool isSuppressed(const ros::Time& timestamp = ros::Time::now()) const;

private:
  Robot* robot_;
  ros::Time last_suppression_timestamp_;
  ros::Time last_accession_timestamp_;
};
}

#endif // _ALLIANCE_ACTIVITY_SUPPRESSION_H_
