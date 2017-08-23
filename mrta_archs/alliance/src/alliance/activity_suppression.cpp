#include "alliance/activity_suppression.h"

namespace alliance
{
ActivitySuppression::ActivitySuppression(const ActivitySuppression &activity_suppression)
{

}

ActivitySuppression::~ActivitySuppression()
{

}

bool ActivitySuppression::suppress(ros::Time timestamp) const
{
  return false;
}
}
