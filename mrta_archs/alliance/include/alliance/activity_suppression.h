#ifndef _ALLIANCE_ACTIVITY_SUPPRESSION_H_
#define _ALLIANCE_ACTIVITY_SUPPRESSION_H_

#include <ros/time.h>
#include <utilities/observer.h>

namespace alliance
{
class Robot;

class ActivitySuppression : public utilities::Observer
{
public:
  ActivitySuppression(Robot* robot);
  ActivitySuppression(const ActivitySuppression& activity_suppression);
  virtual ~ActivitySuppression();
  virtual void update(utilities::Event* event);
  bool suppress(ros::Time timestamp = ros::Time::now()) const;

private:
  Robot* robot_;
};
}

#endif // _ALLIANCE_ACTIVITY_SUPPRESSION_H_
