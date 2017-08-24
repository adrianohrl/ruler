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
  Impatience(double fast_rate);
  Impatience(const Impatience& impatence);
  virtual ~Impatience();
  double getSlowRate(const Robot& robot) const;
  std::map<Robot, double> getSlowRates() const;
  double getFastRate() const;
  ros::Duration getReliabilityDuration(const Robot& robot) const;
  std::map<Robot, ros::Duration> getReliabilityDurations() const;
  double getLevel(ros::Time timestamp = ros::Time::now()) const;
  void setFastRate(double fast_rate);
  void setSlowRate(const Robot& robot, double slow_rate);
  void setReliabilityDuration(const Robot& robot,
                              ros::Duration reliability_duration);

private:
  std::map<Robot, double> slow_rates_;
  double fast_rate_;
  std::map<Robot, ros::Duration> reliability_durations_;
};
}

#endif // _ALLIANCE_IMPATIENCE_H_
