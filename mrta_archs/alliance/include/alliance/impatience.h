#ifndef _ALLIANCE_IMPATIENCE_H_
#define _ALLIANCE_IMPATIENCE_H_

#include "alliance/inter_communication.h"
#include <map>
#include <ros/time.h>
#include <utilities/functions/continuous_sample_holder.h>

namespace alliance
{
class Impatience
{
public:
  Impatience(const RobotPtr& robot, const BehaviourSetPtr& behaviour_set,
             const InterCommunicationPtr& monitor);
  virtual ~Impatience();
  double getSlowRate(const std::string& robot_id,
                     const ros::Time& timestamp = ros::Time::now()) const;
  double getFastRate(const ros::Time& timestamp) const;
  ros::Duration
  getReliabilityDuration(const std::string& robot_id,
                         const ros::Time& timestamp = ros::Time::now()) const;
  double getLevel(const ros::Time& timestamp = ros::Time::now()) const;
  void setSlowRate(const std::string& robot_id, double slow_rate,
                   const ros::Time& timestamp = ros::Time::now());
  void setFastRate(double fast_rate,
                   const ros::Time& timestamp = ros::Time::now());
  void setReliabilityDuration(const std::string& robot_id,
                              const ros::Duration& reliability_duration,
                              const ros::Time& timestamp = ros::Time::now());

private:
  const RobotPtr robot_;
  const InterCommunicationPtr monitor_;
  std::map<std::string, utilities::functions::ContinuousSampleHolderPtr>
      slow_rates_;
  const utilities::functions::ContinuousSampleHolderPtr fast_rate_;
  std::map<std::string, utilities::functions::ContinuousSampleHolderPtr>
      reliability_durations_;
};

typedef boost::shared_ptr<Impatience> ImpatiencePtr;
typedef boost::shared_ptr<Impatience const> ImpatienceConstPtr;
}

#endif // _ALLIANCE_IMPATIENCE_H_
