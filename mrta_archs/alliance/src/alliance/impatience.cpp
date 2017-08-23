#include "alliance/impatience.h"
#include "alliance/robot.h"

namespace alliance
{
Impatience::Impatience(const Impatience& impatence)
    : slow_rates_(impatence.slow_rates_), fast_rate_(impatence.fast_rate_),
      reliability_durations_(impatence.reliability_durations_)
{
}

Impatience::~Impatience() {}

double Impatience::getSlowRate(const Robot& robot) const
{
  //return slow_rates_.at(robot);
}

std::map<Robot, double> Impatience::getSlowRates() const { return slow_rates_; }

double Impatience::getFastRate() const { return fast_rate_; }

ros::Duration Impatience::getReliabilityDuration(const Robot& robot) const
{
  //return reliability_durations_.at(robot);
}

std::map<Robot, ros::Duration> Impatience::getReliabilityDurations() const
{
  return reliability_durations_;
}

double Impatience::getLevel(ros::Time timestamp) const { return 0.0; }
}
