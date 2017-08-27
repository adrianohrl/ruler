#include "alliance/impatience.h"
#include "alliance/robot.h"

namespace alliance
{
Impatience::Impatience(double fast_rate) : fast_rate_(fast_rate)
{
  if (fast_rate <= 0.0)
  {
    throw utilities::Exception("The impatience fast rate must be positive.");
  }
}

Impatience::Impatience(const Impatience& impatence)
    : slow_rates_(impatence.slow_rates_), fast_rate_(impatence.fast_rate_),
      reliability_durations_(impatence.reliability_durations_)
{
}

Impatience::~Impatience() {}

double Impatience::getSlowRate(const Robot& robot) const
{
  // return slow_rates_.at(robot);
}

std::map<Robot, double> Impatience::getSlowRates() const { return slow_rates_; }

double Impatience::getFastRate() const { return fast_rate_; }

ros::Duration Impatience::getReliabilityDuration(const Robot& robot) const
{
  // return reliability_durations_.at(robot);
}

std::map<Robot, ros::Duration> Impatience::getReliabilityDurations() const
{
  return reliability_durations_;
}

double Impatience::getLevel(const ros::Time& timestamp) const { return 0.0; }

void Impatience::setFastRate(double fast_rate)
{
  if (fast_rate <= 0.0)
  {
    throw utilities::Exception("The impatience fast rate must be positive.");
  }
  fast_rate_ = fast_rate;
}

void Impatience::setSlowRate(const Robot& robot, double slow_rate)
{
  if (slow_rate <= 0.0)
  {
    throw utilities::Exception("The impatience fast rate must be positive.");
  }
  // slow_rates_[robot] = slow_rate;
}

void Impatience::setReliabilityDuration(
    const Robot& robot, const ros::Duration& reliability_duration)
{
  // reliability_durations_[robot] = reliability_duration;
}
}
