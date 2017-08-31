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
  Impatience(Robot* robot, BehaviourSet *behaviour_set, InterCommunication* monitor);
  Impatience(const Impatience& impatence);
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
  void setFastRate(double fast_rate, const ros::Time &timestamp = ros::Time::now());
  void setReliabilityDuration(const std::string& robot_id,
                              const ros::Duration& reliability_duration,
                              const ros::Time& timestamp = ros::Time::now());

private:
  Robot* robot_;
  InterCommunication* monitor_;
  std::map<std::string, utilities::functions::ContinuousSampleHolder*>
      slow_rates_;
  utilities::functions::ContinuousSampleHolder* fast_rate_;
  std::map<std::string, utilities::functions::ContinuousSampleHolder*>
      reliability_durations_;
};
}

#endif // _ALLIANCE_IMPATIENCE_H_
