#ifndef _ALLIANCE_IMPATIENCE_H_
#define _ALLIANCE_IMPATIENCE_H_

#include <map>
#include <ros/time.h>

namespace alliance
{
class Robot;

class Impatience
{
public:
  Impatience(const Impatience& impatence);
  virtual ~Impatience();
  double getSlowRate(const Robot& robot) const;
  std::map<Robot, double> getSlowRates() const;
  double getFastRate() const;
  ros::Duration getReliabilityDuration(const Robot& robot) const;
  std::map<Robot, ros::Duration> getReliabilityDurations() const;
  double getLevel(ros::Time timestamp = ros::Time::now()) const;

private:
  std::map<Robot, double> slow_rates_;
  double fast_rate_;
  std::map<Robot, ros::Duration> reliability_durations_;
};
}

#endif // _ALLIANCE_IMPATIENCE_H_