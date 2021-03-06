#ifndef _UTILITIES_NOISY_DURATION_H_
#define _UTILITIES_NOISY_DURATION_H_

#include <ros/duration.h>
#include "utilities/noisy.h"

namespace utilities
{
class NoisyDuration : public Noisy<ros::Duration>
{
public:
  NoisyDuration(const Interval<ros::Duration>& interval);
  NoisyDuration(const ros::Duration& min, const ros::Duration& max);
  NoisyDuration(const ros::Duration& mean, const double& standard_deviation);
  NoisyDuration(const NoisyDuration& noisy);
  virtual ~NoisyDuration();
  virtual double probability(const ros::Duration& d) const;
  virtual ros::Duration random();
  virtual ros::Duration getMean() const;
  virtual Interval<ros::Duration> getFakeInterval() const;
};

typedef boost::shared_ptr<NoisyDuration> NoisyDurationPtr;
typedef boost::shared_ptr<NoisyDuration const> NoisyDurationConstPtr;
}

#endif // _UTILITIES_NOISY_DURATION_H_
