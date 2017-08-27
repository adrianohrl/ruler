#ifndef _ALLIANCE_ACQUIESCENCE_H_
#define _ALLIANCE_ACQUIESCENCE_H_

#include <ros/time.h>

namespace alliance
{
class Acquiescence
{
public:
  Acquiescence(const ros::Duration& yielding_delay,
               const ros::Duration& giving_up_delay);
  Acquiescence(const Acquiescence& acquiescence);
  virtual ~Acquiescence();
  ros::Duration getYieldingDelay() const;
  ros::Duration getGivingUpDelay() const;
  bool isAcquiescent(const ros::Time& timestamp = ros::Time::now());
  void setYieldingDelay(const ros::Duration& yielding_delay);
  void setGivingUpDelay(const ros::Duration& giving_up_delay);

private:
  ros::Duration yielding_delay_;
  ros::Duration giving_up_delay_;
};
}

#endif // _ALLIANCE_ACQUIESCENCE_H_
