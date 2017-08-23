#ifndef _ALLIANCE_ACTIVITY_SUPPRESSION_H_
#define _ALLIANCE_ACTIVITY_SUPPRESSION_H_

#include <ros/time.h>

namespace alliance
{
class ActivitySuppression
{
public:
  ActivitySuppression(const ActivitySuppression& activity_suppression);
  virtual ~ActivitySuppression();
  bool suppress(ros::Time timestamp = ros::Time::now()) const;

private:
};
}

#endif // _ALLIANCE_ACTIVITY_SUPPRESSION_H_
