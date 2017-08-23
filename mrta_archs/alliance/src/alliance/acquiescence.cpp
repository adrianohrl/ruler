#include "alliance/acquiescence.h"

namespace alliance
{
Acquiescence::Acquiescence(const Acquiescence& acquiescence)
    : yielding_delay_(acquiescence.yielding_delay_),
      giving_up_delay_(acquiescence.giving_up_delay_)
{
}

Acquiescence::~Acquiescence() {}

ros::Duration Acquiescence::getYieldingDelay() const { return yielding_delay_; }

ros::Duration Acquiescence::getGivingUpDelay() const
{
  return giving_up_delay_;
}

bool Acquiescence::isAcquiescent(ros::Time timestamp) { return false; }

void Acquiescence::setYieldingDelay(ros::Duration yielding_delay)
{
  yielding_delay_ = yielding_delay;
}

void Acquiescence::setGivingUpDelay(ros::Duration giving_up_delay)
{
  giving_up_delay_ = giving_up_delay;
}
}
