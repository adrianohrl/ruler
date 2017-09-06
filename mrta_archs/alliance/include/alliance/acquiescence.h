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
  Acquiescence(const RobotPtr& robot,
               const BehaviourSetPtr& behaviour_set,
               const InterCommunicationPtr& monitor);
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
  const RobotPtr robot_;
  const BehaviourSetPtr behaviour_set_;
  const InterCommunicationPtr monitor_;
  const utilities::functions::ContinuousSampleHolderPtr yielding_delay_;
  const utilities::functions::ContinuousSampleHolderPtr giving_up_delay_;
};

typedef boost::shared_ptr<Acquiescence> AcquiescencePtr;
typedef boost::shared_ptr<Acquiescence const> AcquiescenceConstPtr;
}

#endif // _ALLIANCE_ACQUIESCENCE_H_
