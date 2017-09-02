#ifndef _ALLIANCE_ACQUIESCENCE_H_
#define _ALLIANCE_ACQUIESCENCE_H_

#include "alliance/inter_communication.h"
#include <ros/time.h>
#include <utilities/functions/continuous_sample_holder.h>

namespace alliance
{
class Acquiescence
{
public:
  Acquiescence(Robot* robot, BehaviourSet* behaviour_set,
               InterCommunication* monitor);
  virtual ~Acquiescence();
  ros::Duration
  getYieldingDelay(const ros::Time& timestamp = ros::Time::now()) const;
  ros::Duration
  getGivingUpDelay(const ros::Time& timestamp = ros::Time::now()) const;
  bool isAcquiescent(const ros::Time& timestamp = ros::Time::now());
  void setYieldingDelay(const ros::Duration& yielding_delay,
                        const ros::Time& timestamp = ros::Time::now());
  void setGivingUpDelay(const ros::Duration& giving_up_delay,
                        const ros::Time& timestamp = ros::Time::now());

private:
  Robot* robot_;
  BehaviourSet* behaviour_set_;
  InterCommunication* monitor_;
  utilities::functions::ContinuousSampleHolder* yielding_delay_;
  utilities::functions::ContinuousSampleHolder* giving_up_delay_;
};
}

#endif // _ALLIANCE_ACQUIESCENCE_H_
